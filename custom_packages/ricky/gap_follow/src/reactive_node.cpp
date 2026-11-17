// reactive_node.cpp
// Lab 4: ghost-car obstacle avoidance, paired with a wall follower, with a
// slow precise mode for tight gaps.
//
// THE IDEA (borrowed from our emergency-brake node)
// -------------------------------------------------
// Every plan, send out "ghost cars": see-through copies of the car, each
// driving a different path the real car could take. At every step along its
// path the ghost's WHOLE BODY is checked against the LiDAR points. The
// distance a ghost gets before touching something is its "free distance".
// The car follows the best ghost for 20 ms, then plans again from scratch.
//
// TWO DRIVING MODES
//
// CRUISE (open hallways): fast and smooth.
//   Ghosts:  turn ghosts (swing to a heading, -90..+90 deg, then straight) and
//            swerve ghosts (an S-bend lane change, then straight along the
//            hallway), driven up to 4 m.
//   Score:   free distance - squeeze penalty (tight paths cost more)
//            - distance from the wall advisor's heading - lane change
//            - change of mind.
//
// THREAD (tight spots, e.g. the three-box slalom): slow and precise.
//   Entered when the best cruise path is blocked within thread_enter_free or
//   squeezes within thread_enter_clearance; left once paths are wide open.
//   Ghosts:  every pair of steering angles (one for the first thread_split m,
//            another for the rest), driven thread_horizon m. Between them
//            they draw slight diagonals, small jogs and S-bends, which a
//            single turn cannot.
//   Rule:    any ghost that touches something (body + thread_margin) is out.
//   Score:   stay farthest from everything (aim for the middle of the gap)
//            + get somewhere - end up off the advisor's direction
//            - change the plan - try the side that just failed again.
//   Speed:   thread_speed, so the car follows the plan to within a few cm.
//
// THE WALL ADVISOR (same math as Lab 3) suggests a preferred heading:
//   - wall beside us        -> the heading that holds wall_distance
//   - wall reading is junk  -> no opinion. "Junk" = the two beams disagree
//                              (one fell into a recess or onto an obstacle),
//                              which shows up as a large wall angle.
//   - a real hallway opened -> turn into it. "Real" = a ghost can turn into it
//     on the wall side         and drive wall_open_depth with room to spare.
//
// SPEED: never faster than the car could still stop within the chosen ghost's
// free distance, slower in turns (grip), and already down to turning speed
// by the time something ahead is turn_room away.
//
// BACKING UP: if no ghost can move forward, the car stops, then reverses along
// the path it just drove (floor known to be clear). The LiDAR cannot see
// behind the car (its view is 270 deg), so the back-up is also checked against
// dots seen in the last memory_time seconds, stops early enough to allow for
// rolling, never goes past the recorded path, and gets longer if the car stays
// stuck. The next THREAD attempt avoids the side that just failed.
//
// DIRECTION: the node has no built-in direction; the car drives the way it
// is facing when it starts. wall_side picks the wall the advisor tracks. On the
// Levine maps "right" works both ways: the inner wall when driving clockwise,
// the outer wall counter-clockwise (that lines the car up for the south-east
// corner of levine_obs).
//
// Reads only /scan and publishes only /drive (plus its own marker topic).
// It knows its speed and motion from what it commanded, not from odometry.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ==== CORE BEGIN (plain C++, no ROS: all of the planning) =====================
namespace gap
{
constexpr double kPi = 3.14159265358979323846;
inline double radians(double deg) { return deg * kPi / 180.0; }
inline double degrees(double rad) { return rad * 180.0 / kPi; }

struct Params
{
  // ---- Car shape, measured from the LiDAR (defaults: the simulator's car) ----
  double car_half_width = 0.155;      // m
  double lidar_to_front = 0.015;      // LiDAR -> front bumper (m)
  double lidar_to_rear = 0.565;       // LiDAR -> rear bumper (m)
  double lidar_to_rear_axle = 0.446;  // LiDAR -> rear axle, the point the car turns about (m)
  double wheelbase = 0.33;            // m
  double max_steering = 0.34;         // rad

  // ---- Margins around the ghost's body ----------------------------------------
  double side_margin = 0.20;    // comfort margin on each side (m)
  double hard_margin = 0.05;    // contact margin on each side (m)
  double front_margin = 0.05;   // extra room in front (m)
  double margin_spread = 0.5;   // margins grow from 0 by this many m per m traveled

  // ---- Ghost paths --------------------------------------------------------------
  double ghost_step = 0.10;      // m between ghost snapshots
  double reaction_time = 0.05;   // s before a new steering command takes hold
  double steer_rate = 3.2;       // rad/s, how fast the wheels can swing over
  double speed_up_time = 0.20;   // s the car takes to pick up a higher commanded speed
  double slow_down_time = 0.05;  // s it takes to drop to a lower one
  double radius_scale = 1.0;     // > 1 = assume the car turns wider than the geometry says
  double lookahead_time = 1.0;   // a heading must be clear for speed x this ...
  double lookahead_min = 1.5;    // ... but at least this (m) ...
  double lookahead_max = 4.0;    // ... and at most this (m)
  double heading_limit_deg = 90.0;
  double heading_step_deg = 5.0;
  double swerve_max = 1.0;       // swerve ghosts move over sideways by up to this (m) ...
  double swerve_step = 0.1;      // ... in steps of this (m)

  // ---- Scoring (meters of free distance "charged" per radian) -------------------
  double squeeze_weight = 3.0;   // cost (m of free distance) per m a path is tighter than side_margin
  double turn_weight = 0.6;      // pull toward the preferred heading
  double swerve_weight = 0.3;    // cost per m of sideways move (staying in lane is cheaper)
  double change_weight = 0.3;    // reluctance to change heading (keep below turn_weight)
  double change_cap_deg = 30.0;  // ... counted up to this much change
  double heading_filter_tau = 0.05;  // s, smoothing of the steering command
  double steer_gain = 1.0;       // steering angle = gain x chosen heading

  // ---- Speed --------------------------------------------------------------------
  double max_speed = 3.0;
  double min_speed = 0.5;
  double brake_decel = 4.0;         // m/s^2 the car can count on when slowing
  double lateral_accel = 5.0;       // m/s^2 allowed sideways in a turn
  double stop_buffer = 0.25;        // m kept in hand when stopping for something
  double emergency_distance = 0.15; // chosen ghost blocked closer than this = stop
  double turn_room = 2.0;           // be at turning speed when this far from a blockage

  // ---- Wall advisor ---------------------------------------------------------------
  double wall_distance = 0.75;
  double wall_kp = 1.0;               // tuned in Lab 3
  double wall_lookahead = 0.5;
  double wall_theta_deg = 50.0;
  double wall_beam_window_deg = 1.0;
  double wall_approach_limit_deg = 6.0;  // gentle toward the wall ...
  double wall_away_limit_deg = 20.0;     // ... firmer away from it
  double wall_max_angle_deg = 15.0;      // beyond this the beams aren't on one wall
  double wall_open_depth = 3.0;          // a ghost must get this far into an opening
  double wall_turn_min_deg = 60.0;       // ... on a heading at least this steep
  double wall_turn_extra_margin = 0.10;  // ... with this much extra side room

  // ---- Backing up -------------------------------------------------------------------
  double reverse_speed = 0.5;     // m/s
  double reverse_distance = 0.4;  // m for the first back-up; each repeat in the same spot adds this much
  double reverse_distance_max = 1.6;  // ... up to this (m)
  double reverse_clearance = 0.25;    // stop backing up with at least this much room behind (m)
  double reverse_brake_decel = 1.0;   // m/s^2 the car slows at when stopping a back-up
  double memory_time = 6.0;           // s: remember nearby dots this long, for checking back-ups
  double memory_radius = 1.5;         // m: ... the ones within this of the car

  // ---- THREAD mode: slow, precise driving through tight gaps -----------------------
  double thread_enter_free = 1.5;       // enter when the best cruise path is blocked within this (m) ...
  double thread_enter_clearance = 0.08; // ... or squeezes closer than this to something (m)
  double thread_exit_clearance = 0.15;  // leave once paths are fully open with this much room (m)
  double thread_speed = 0.5;            // m/s
  double thread_margin = 0.02;          // contact margin on each side in THREAD mode (m)
  double thread_horizon = 1.8;          // how far each THREAD ghost drives (m)
  double thread_split = 0.4;            // THREAD ghosts change steering after this far (m)
  double thread_step = 0.05;            // m between THREAD ghost snapshots
  double thread_clear_weight = 10.0;    // score per m of clearance (stay in the middle of the gap) ...
  double thread_clear_cap = 0.25;       // ... counted up to this much clearance (m)
  double thread_progress_weight = 1.0;  // score for getting somewhere (1 = full horizon)
  double thread_align_weight = 0.5;     // cost per rad of ending up off the advisor's direction
  double thread_commit_weight = 0.5;    // reluctance to change the plan
  double thread_failed_weight = 1.0;    // cost of trying again on the side that just failed

  // ---- Switches / counts (not in the table below) ---------------------------------
  bool use_thread = true;
  int thread_exit_plans = 10;  // plans in a row of open space before leaving THREAD mode
  bool follow_wall = true;
  double wall_side = -1.0;   // -1 = right wall, +1 = left wall
  bool back_up = true;
  int beam_skip = 4;         // use every Nth LiDAR beam for the ghost checks
  int blocked_plans = 5;     // plans in a row with no way forward before backing up
};

// Name -> field table for every decimal parameter (used to declare them and
// to update them live).
struct ParamRef { const char * name; double Params::* field; };
inline const std::vector<ParamRef> & doubleParams()
{
  static const std::vector<ParamRef> table = {
    {"car_half_width", &Params::car_half_width},
    {"lidar_to_front", &Params::lidar_to_front},
    {"lidar_to_rear", &Params::lidar_to_rear},
    {"lidar_to_rear_axle", &Params::lidar_to_rear_axle},
    {"wheelbase", &Params::wheelbase},
    {"max_steering", &Params::max_steering},
    {"side_margin", &Params::side_margin},
    {"hard_margin", &Params::hard_margin},
    {"front_margin", &Params::front_margin},
    {"margin_spread", &Params::margin_spread},
    {"ghost_step", &Params::ghost_step},
    {"reaction_time", &Params::reaction_time},
    {"steer_rate", &Params::steer_rate},
    {"speed_up_time", &Params::speed_up_time},
    {"slow_down_time", &Params::slow_down_time},
    {"radius_scale", &Params::radius_scale},
    {"lookahead_time", &Params::lookahead_time},
    {"lookahead_min", &Params::lookahead_min},
    {"lookahead_max", &Params::lookahead_max},
    {"heading_limit_deg", &Params::heading_limit_deg},
    {"heading_step_deg", &Params::heading_step_deg},
    {"swerve_max", &Params::swerve_max},
    {"swerve_step", &Params::swerve_step},
    {"swerve_weight", &Params::swerve_weight},
    {"turn_weight", &Params::turn_weight},
    {"change_weight", &Params::change_weight},
    {"change_cap_deg", &Params::change_cap_deg},
    {"heading_filter_tau", &Params::heading_filter_tau},
    {"steer_gain", &Params::steer_gain},
    {"max_speed", &Params::max_speed},
    {"min_speed", &Params::min_speed},
    {"brake_decel", &Params::brake_decel},
    {"lateral_accel", &Params::lateral_accel},
    {"stop_buffer", &Params::stop_buffer},
    {"emergency_distance", &Params::emergency_distance},
    {"turn_room", &Params::turn_room},
    {"wall_distance", &Params::wall_distance},
    {"wall_kp", &Params::wall_kp},
    {"wall_lookahead", &Params::wall_lookahead},
    {"wall_theta_deg", &Params::wall_theta_deg},
    {"wall_beam_window_deg", &Params::wall_beam_window_deg},
    {"wall_approach_limit_deg", &Params::wall_approach_limit_deg},
    {"wall_away_limit_deg", &Params::wall_away_limit_deg},
    {"wall_max_angle_deg", &Params::wall_max_angle_deg},
    {"wall_open_depth", &Params::wall_open_depth},
    {"wall_turn_min_deg", &Params::wall_turn_min_deg},
    {"wall_turn_extra_margin", &Params::wall_turn_extra_margin},
    {"reverse_speed", &Params::reverse_speed},
    {"reverse_distance", &Params::reverse_distance},
    {"reverse_distance_max", &Params::reverse_distance_max},
    {"reverse_clearance", &Params::reverse_clearance},
    {"reverse_brake_decel", &Params::reverse_brake_decel},
    {"memory_time", &Params::memory_time},
    {"memory_radius", &Params::memory_radius},
    {"squeeze_weight", &Params::squeeze_weight},
    {"thread_enter_free", &Params::thread_enter_free},
    {"thread_enter_clearance", &Params::thread_enter_clearance},
    {"thread_exit_clearance", &Params::thread_exit_clearance},
    {"thread_speed", &Params::thread_speed},
    {"thread_margin", &Params::thread_margin},
    {"thread_horizon", &Params::thread_horizon},
    {"thread_split", &Params::thread_split},
    {"thread_step", &Params::thread_step},
    {"thread_clear_weight", &Params::thread_clear_weight},
    {"thread_clear_cap", &Params::thread_clear_cap},
    {"thread_progress_weight", &Params::thread_progress_weight},
    {"thread_align_weight", &Params::thread_align_weight},
    {"thread_commit_weight", &Params::thread_commit_weight},
    {"thread_failed_weight", &Params::thread_failed_weight},
  };
  return table;
}

// The bits of a LaserScan the planner needs
struct ScanView
{
  const std::vector<float> & ranges;
  double angle_min, angle_inc, range_min, range_max;
};

// Where a ghost's rear axle is (in the LiDAR's frame: x forward, y left) and
// which way the ghost faces (cos / sin of its heading)
struct Pose { double x, y, c, s; };

// How far a ghost gets (real contact / comfort margin) and the tightest side
// clearance it has on the way (m, measured over the near part of the path)
struct Free { double hard, comfort, clearance; };

// One ghost car's plan. Either
//   a turn:   swing to `heading` (rad, + = left), then drive straight, or
//   a swerve: move over sideways by `shift` (m, + = left) with an S-bend,
//             then carry on straight along the hallway.
struct Ghost { double heading, shift; };

// A ghost's path as geometry: four pieces, each an arc (curvature in 1/m,
// + = left, 0 = straight) of some length (m), followed by a straight line.
//   piece 0: what the car does while the wheels swing over to the new angle
//            (it keeps going on its current arc)
//   piece 1: the first arc of the ghost's plan
//   piece 2: swerves only: the stretch covered while the wheels swing from
//            full lock one way to full lock the other (roughly straight)
//   piece 3: swerves only: the arc back
struct Piece { double curvature, length; };
struct Path { Piece piece[4]; };
struct Result { Ghost ghost; Free free; };

struct Output
{
  double steer = 0.0;
  double speed = 0.0;
  const char * mode = "straight";  // what the advisor is doing
  double preferred = 0.0;          // heading the advisor asked for (rad)
  double heading = 0.0;            // chosen ghost: heading it turns to (rad) ...
  double shift = 0.0;              // ... or sideways move it makes (m)
  double free = 0.0;               // how far the chosen ghost gets (m)
  double wall_distance = 0.0;      // m
  bool reversing = false;
  bool started_reverse = false;
  double reverse_target = 0.0;     // how far this back-up will go (m)
  bool boxed_in = false;           // no way forward and no room to back up
  int ghosts = 0;                  // how many ghost cars were driven this plan
  bool thread = false;             // THREAD mode was in charge of this plan
};

// What one THREAD ghost found out
struct Thread
{
  double steer1 = 0.0, steer2 = 0.0;  // its two steering angles (rad)
  double free = 0.0;        // how far it gets before contact (m)
  double clearance = 0.0;   // closest it comes to anything on the way (m)
  double end_x = 0.0, end_y = 0.0, end_heading = 0.0;  // where it ends up
};

class Planner
{
public:
  Params p;

  // One planning cycle. dt = seconds since the previous cycle.
  Output plan(const ScanView & scan, double dt)
  {
    Output out;
    ghost_count_ = 0;
    const double length = std::max({p.lookahead_max, p.wall_open_depth, p.turn_room + 1.0});
    collectPoints(scan, length);
    remember(dt);

    // ---- Backing up: retrace the last stretch we drove --------------------------
    if (reversing_) {
      trackSpeed(-p.reverse_speed, dt);
      const double step = std::max(0.0, -speed_estimate_) * dt;  // distance actually covered
      reversed_ += step;
      net_progress_ -= step;
      // Replay the forward steering, newest first, over the same distances.
      // That floor was clear a moment ago, which matters because the LiDAR
      // cannot see straight behind the car.
      double steer = 0.0;
      double left = step;
      while (left > 0.0 && !trail_.empty()) {
        steer = trail_.back().steer;
        if (trail_.back().distance > left) {
          trail_.back().distance -= left;
          trail_length_ -= left;
          left = 0.0;
        } else {
          left -= trail_.back().distance;
          trail_length_ -= trail_.back().distance;
          trail_.pop_back();
        }
      }
      // Backing up with the wheels turned swings the car the other way.
      lane_heading_ -= speed_estimate_ * std::tan(steer) / std::max(0.01, p.wheelbase) * dt;
      last_cmd_steer_ = steer;
      // Stop with room to spare: the car keeps rolling back a little after
      // the command changes (v^2 / 2a), so end the back-up that much early.
      const double rolling =
        p.reverse_speed * p.reverse_speed / (2.0 * std::max(0.1, p.reverse_brake_decel));
      // Also stop once the recorded path runs out: beyond it is floor we
      // never drove on, in the LiDAR's blind spot behind the car.
      if (reversed_ + rolling >= reverse_target_ || trail_length_ < rolling ||
          reverseFree(steer) < p.reverse_clearance + rolling) {
        reversing_ = false;
        blocked_ = 0;
        steer_ = 0.0;
        last_speed_ = 0.0;
      } else {
        out.steer = steer;
        out.speed = -p.reverse_speed;
        out.mode = "BACKING UP";
        out.reversing = true;
        return out;
      }
    }

    // ---- 1. Send out the ghosts ----------------------------------------------------------
    const double cap = std::clamp(last_speed_ * p.lookahead_time, p.lookahead_min,
                                  std::max(p.lookahead_min, p.lookahead_max));
    // Which way does the hallway run, relative to the way we're pointing?
    // The wall beams tell us when they sit on one wall; in between we keep
    // track by adding up our own turning.
    lane_heading_ -= speed_estimate_ * std::tan(steer_) / std::max(0.01, p.wheelbase) * dt;
    bool wall_trusted = false;
    double wall_alpha = 0.0;
    const double wall_wish = wallHeading(scan, wall_trusted, out.wall_distance, wall_alpha);
    if (wall_trusted) lane_heading_ = p.wall_side * wall_alpha;
    if (std::abs(lane_heading_) > radians(75.0)) lane_heading_ = 0.0;  // we've turned a corner

    fan_.clear();
    // (a) Turn ghosts: swing to a heading, then go straight.
    const double step = radians(std::max(1.0, p.heading_step_deg));
    const int n = static_cast<int>(std::lround(radians(std::clamp(p.heading_limit_deg, 0.0, 120.0)) / step));
    for (int k = -n; k <= n; ++k) fan_.push_back(drive({k * step, 0.0}, length, p.side_margin, cap));
    if (std::abs(lane_heading_) > 1e-3) {
      fan_.push_back(drive({lane_heading_, 0.0}, length, p.side_margin, cap));  // straighten up
    }
    // (b) Swerve ghosts: move over sideways (an S-bend), then carry on straight
    //     along the hallway. A turn ghost cannot show a way through that needs
    //     a lane change, such as the open lane beside a dead-end pocket.
    const double swerve_step = std::max(0.05, p.swerve_step);
    const int swerves = static_cast<int>(std::lround(std::max(0.0, p.swerve_max) / swerve_step));
    for (int k = 1; k <= swerves; ++k) {
      fan_.push_back(drive({0.0, k * swerve_step}, length, p.side_margin, cap));
      fan_.push_back(drive({0.0, -k * swerve_step}, length, p.side_margin, cap));
    }

    // ---- 2. The wall advisor's preferred heading ---------------------------------------
    double preferred = 0.0;
    if (p.follow_wall) {
      preferred = wall_wish;
      out.mode = wall_trusted ? "wall" : "no wall";

      // A real hallway on the wall side? Send ghosts down the steep headings
      // with extra side room; take the one that gets furthest (ties: least steep).
      const double turn_min = radians(p.wall_turn_min_deg) - 1e-9;
      bool found = false;
      double turn = 0.0, turn_free = 0.0;
      for (int k = -n; k <= n; ++k) {
        const double h = k * step;
        if (p.wall_side * h < turn_min) continue;
        const double f =
          drive({h, 0.0}, length, p.side_margin + p.wall_turn_extra_margin, cap).free.comfort;
        if (f < p.wall_open_depth) continue;
        const bool better = !found || f > turn_free + 1e-9 ||
          (std::abs(f - turn_free) <= 1e-9 && std::abs(h) < std::abs(turn));
        if (better) { found = true; turn = h; turn_free = f; }
      }
      if (found) { preferred = turn; out.mode = "turn-in"; }
    }

    // ---- 3. Score every ghost (plus one on the advisor's exact heading) --------------------
    fan_.push_back(drive({preferred, 0.0}, length, p.side_margin, cap));
    // How open the preferred way is (1 = fully). Once it is blocked within
    // our lookahead its pull fades to nothing: we will have to leave it anyway.
    const double open_ratio =
      std::clamp((std::min(fan_.back().free.hard, cap) / cap - 0.75) / 0.25, 0.0, 1.0);
    const double pull = p.turn_weight * open_ratio;
    const double roomy = p.side_margin + p.wall_turn_extra_margin;
    const double change_cost = p.change_weight * radians(p.change_cap_deg);
    double best_score = -std::numeric_limits<double>::infinity();
    std::size_t best = 0;
    for (std::size_t i = 0; i < fan_.size(); ++i) {
      const Result & r = fan_[i];
      const double score =
        std::min(r.free.hard, cap)                                       // how far it gets
        - p.squeeze_weight * std::max(0.0, roomy - r.free.clearance)     // how tight it is
        - pull * std::abs(finalHeading(r.ghost) - preferred)             // off the advisor's heading
        - p.swerve_weight * std::abs(r.ghost.shift)                      // leaving our lane
        - change_cost * std::min(                                        // changing our mind
            std::abs(firstSteer(r.ghost) - steer_) / std::max(0.05, p.max_steering), 1.5);
      if (score > best_score) { best_score = score; best = i; }
    }
    const Result chosen = fan_[best];
    chosen_path_ = pathFor(chosen.ghost);
    double target_steer = firstSteer(chosen.ghost);

    // ---- 3b. THREAD mode: tight spot? Switch to slow, precise driving -------------------------
    // The cruise ghosts turn once and then drive straight. In a tight cluster
    // the way through often needs a slight diagonal or a small jog, which none
    // of them can draw. THREAD ghosts steer one way, then another.
    if (p.use_thread) {
      const bool tight = chosen.free.hard < p.thread_enter_free ||
                         chosen.free.clearance < p.thread_enter_clearance;
      if (!thread_ && tight) {
        thread_ = true;
        open_plans_ = 0;
      } else if (thread_) {
        const bool open = !tight && chosen.free.hard >= cap - 1e-6 &&
                          chosen.free.clearance >= p.thread_exit_clearance;
        open_plans_ = open ? open_plans_ + 1 : 0;
        if (open_plans_ >= std::max(1, p.thread_exit_plans)) thread_ = false;
      }
    } else {
      thread_ = false;
    }
    Thread pick;
    if (thread_) {
      // Line up the way the wall advisor wants when it trusts the wall (good
      // corner entries); otherwise just along the hallway.
      pick = threadPlan(p.follow_wall && wall_trusted ? preferred : lane_heading_);
      target_steer = pick.steer1;
      out.mode = "THREAD";
      out.thread = true;
    }

    // ---- 4. Steer the way the winning ghost starts out (smoothed so it can't jump) ----------
    steer_ += dt / (std::max(0.0, p.heading_filter_tau) + dt) * (target_steer - steer_);
    const double steer = std::clamp(steer_, -p.max_steering, p.max_steering);

    // ---- 5. Speed ---------------------------------------------------------------------------------
    // How far the winning ghost gets before real contact.
    const double free_now = thread_ ? pick.free : chosen.free.hard;
    // (a) Could still stop in time: v = sqrt(2 a d).
    const double v_brake =
      std::sqrt(std::max(0.0, 2.0 * p.brake_decel * (free_now - p.stop_buffer)));
    // (b) Grip in the turn we're in: v = sqrt(a_lat * R), R = wheelbase / tan(steer).
    const double curvature = std::tan(std::abs(steer)) / std::max(0.01, p.wheelbase);
    const double v_corner = curvature > 1e-6
      ? std::sqrt(std::max(0.0, p.lateral_accel) / curvature) : p.max_speed;
    // (c) Something ahead: be down to full-lock turning speed turn_room before it.
    const double tightest = p.wheelbase / std::tan(std::max(0.05, p.max_steering));
    const double v_turn2 = std::max(0.0, p.lateral_accel) * tightest;
    const double v_ahead =
      std::sqrt(v_turn2 + 2.0 * p.brake_decel * std::max(0.0, free_now - p.turn_room));
    double speed = std::max(std::min({p.max_speed, v_brake, v_corner, v_ahead}), p.min_speed);
    if (thread_) {
      // Slow and precise: at thread_speed the car follows a plan to within a
      // few cm. Still never faster than it could stop in.
      const double crawl = std::min(p.thread_speed, p.min_speed);
      speed = std::max(std::min({p.thread_speed, p.max_speed, v_brake}), crawl);
    }

    // ---- 6. No way forward: stop, then back up ---------------------------------------------------------
    if (free_now < p.emergency_distance) {
      speed = 0.0;
      ++blocked_;
      if (blocked_ >= std::max(1, p.blocked_plans)) {
        out.boxed_in = true;
        if (p.back_up) {
          const double first_steer = trail_.empty() ? 0.0 : trail_.back().steer;
          const double rolling =
            p.reverse_speed * p.reverse_speed / (2.0 * std::max(0.1, p.reverse_brake_decel));
          if (trail_length_ >= rolling + 0.10 &&
              reverseFree(first_steer) >= p.reverse_clearance + rolling) {
            // Stuck again without getting anywhere? Back up further each time,
            // so the next try starts from a different view of the obstacles.
            if (net_progress_ > 1.0) { stuck_streak_ = 0; failed_side_ = 0.0; }
            ++stuck_streak_;
            // Remember which side we were trying, so the next THREAD attempt
            // leans toward the other one instead of repeating the same mistake.
            if (thread_ && std::abs(pick.end_y) > 0.05) failed_side_ = pick.end_y > 0.0 ? 1.0 : -1.0;
            thread_ = true;  // come back out of the back-up in THREAD mode
            open_plans_ = 0;
            net_progress_ = 0.0;
            reverse_target_ = std::min({p.reverse_distance * stuck_streak_,
                                        std::max(p.reverse_distance, p.reverse_distance_max),
                                        trail_length_});
            reversing_ = true;
            reversed_ = 0.0;
            out.started_reverse = true;
            out.reverse_target = reverse_target_;
            out.boxed_in = false;
          }
        }
      }
    } else {
      blocked_ = 0;
    }

    // Remember what we just did, so a back-up can retrace it. Distances use
    // our estimate of the real speed, which lags behind the commanded one.
    trackSpeed(speed, dt);
    if (speed_estimate_ > 0.0) {
      const double moved = speed_estimate_ * dt;
      trail_.push_back({steer, moved});
      trail_length_ += moved;
      net_progress_ += moved;
      if (net_progress_ > 1.5) failed_side_ = 0.0;  // got somewhere: forget the failure
      while (trail_length_ > p.reverse_distance_max + 1.0 && trail_.size() > 1) {
        trail_length_ -= trail_.front().distance;
        trail_.pop_front();
      }
    }

    last_speed_ = speed;
    last_cmd_steer_ = steer;
    out.steer = steer;
    out.speed = speed;
    out.preferred = preferred;
    out.heading = chosen.ghost.heading;
    out.shift = chosen.ghost.shift;
    out.free = free_now;
    out.ghosts = ghost_count_;
    return out;
  }

  // Where the path the car is following (the winning ghost) is after
  // `distance` m. For drawing it in Foxglove.
  Pose plannedPose(double distance) const { return poseOn(chosen_path_, distance); }
  bool threading() const { return thread_; }

  // Turn a ghost's plan into path geometry.
  Path pathFor(const Ghost & g) const
  {
    const double wheelbase = std::max(0.01, p.wheelbase);
    // Piece 0: the wheels can't jump to a new angle. While they swing over,
    // the car carries on along the arc it is on now. (Treated as: hold the
    // current angle for the reaction time plus half the swing time.)
    const double target = firstSteer(g, 0.0);
    const double swing_time = p.reaction_time +
      0.5 * std::abs(target - steer_) / std::max(0.1, p.steer_rate);
    const Piece lead{std::tan(steer_) / wheelbase / p.radius_scale,
                     std::max(0.0, last_speed_) * swing_time};
    const double turned = lead.curvature * lead.length;  // heading change during piece 0

    if (std::abs(g.shift) > 1e-6) {
      // Swerve: arc toward the new lane, arc back, end up pointing along the
      // hallway, `shift` to the side of where we are now.
      const double tight = tightestCurvature();
      const double s = g.shift > 0.0 ? 1.0 : -1.0;
      // u: how far we already point toward the side we're moving to.
      const double u = -s * (lane_heading_ - turned);
      const double between = swingOverDistance();
      const double swing = swerveSwing(std::abs(g.shift), u, tight, between);
      const double out_angle = std::max(0.0, swing - u);
      return {{lead, {s * tight, out_angle / tight},
               {0.0, out_angle > 1e-6 ? between : 0.0}, {-s * tight, swing / tight}}};
    }
    // Turn: one arc, at the steering the car would use, to the new heading.
    const double remaining = g.heading - turned;
    const double steer = std::min(std::abs(p.steer_gain * g.heading), p.max_steering);
    const double curvature = std::max(1e-6, std::tan(steer) / wheelbase / p.radius_scale);
    const double sign = remaining > 0.0 ? 1.0 : -1.0;
    return {{lead, {sign * curvature, std::abs(remaining) / curvature}, {0.0, 0.0}, {0.0, 0.0}}};
  }

  // Where a ghost is after driving `distance` along a path: the rear axle's
  // position in the LiDAR's frame, and the ghost's heading.
  Pose poseOn(const Path & path, double distance) const
  {
    double x = -p.lidar_to_rear_axle;
    double y = 0.0;
    double heading = 0.0;
    double left = distance;
    for (const Piece & piece : path.piece) {
      const double d = std::min(left, piece.length);
      if (d <= 0.0) continue;
      if (std::abs(piece.curvature) < 1e-9) {
        x += d * std::cos(heading);
        y += d * std::sin(heading);
      } else {
        const double next = heading + piece.curvature * d;
        x += (std::sin(next) - std::sin(heading)) / piece.curvature;
        y -= (std::cos(next) - std::cos(heading)) / piece.curvature;
        heading = next;
      }
      left -= d;
    }
    x += left * std::cos(heading);  // the rest is straight
    y += left * std::sin(heading);
    return {x, y, std::cos(heading), std::sin(heading)};
  }

  Pose ghostPose(const Ghost & g, double distance) const { return poseOn(pathFor(g), distance); }

  // The steering angle a ghost starts out with: what the car should do now
  // to follow it. `turned` = heading change already made before its first arc.
  double firstSteer(const Ghost & g, double turned = 0.0) const
  {
    if (std::abs(g.shift) > 1e-6) {
      const double s = g.shift > 0.0 ? 1.0 : -1.0;
      const double u = -s * (lane_heading_ - turned);
      const double swing =
        swerveSwing(std::abs(g.shift), u, tightestCurvature(), swingOverDistance());
      // Normally start by steering toward the new lane; if we already point
      // that way more than needed, start by steering back.
      return (swing > u + 1e-3 ? s : -s) * p.max_steering;
    }
    return std::clamp(p.steer_gain * g.heading, -p.max_steering, p.max_steering);
  }

  // The tightest turn the car can make, as curvature (1/m).
  double tightestCurvature() const
  {
    return std::tan(std::max(0.05, p.max_steering)) / std::max(0.01, p.wheelbase) / p.radius_scale;
  }

  // Distance covered while the wheels swing from full lock to full lock.
  double swingOverDistance() const
  {
    return std::max(0.0, last_speed_) * 2.0 * p.max_steering / std::max(0.1, p.steer_rate);
  }

  // How far a swerve has to swing out (peak angle to the hallway, rad) to end
  // up `shift` to the side. u = how far we already point toward that side.
  // Sideways distance gained = first arc + the swing-over stretch + arc back.
  double swerveSwing(double shift, double u, double tight, double between) const
  {
    const auto gained = [&](double swing) {
      return (std::cos(u) - std::cos(swing)) / tight + between * std::sin(swing)
             + (1.0 - std::cos(swing)) / tight;
    };
    double low = std::max(u, 0.0);
    double high = kPi / 2.0;
    if (gained(low) >= shift) return low;
    if (gained(high) <= shift) return high;
    for (int i = 0; i < 24; ++i) {
      const double mid = 0.5 * (low + high);
      (gained(mid) < shift ? low : high) = mid;
    }
    return 0.5 * (low + high);
  }

  // The direction a ghost ends up driving in (rad, relative to the car now).
  double finalHeading(const Ghost & g) const
  {
    return std::abs(g.shift) > 1e-6 ? lane_heading_ : g.heading;
  }

  double lastSpeed() const { return last_speed_; }

  // Our best guess of the car's real speed: it follows the commanded speed
  // with a short delay (no odometry here, so we model it).
  void trackSpeed(double commanded, double dt)
  {
    const double tau = std::abs(commanded) > std::abs(speed_estimate_)
      ? p.speed_up_time : p.slow_down_time;
    speed_estimate_ += dt / (std::max(0.0, tau) + dt) * (commanded - speed_estimate_);
  }

  // Every ghost from the last plan and how it did (for debugging).
  const std::vector<Result> & fan() const { return fan_; }

private:
  struct Point { double x, y; };

  // LiDAR beams -> floor points near enough to matter. Points inside our own
  // body are the car seeing itself: dropped.
  void collectPoints(const ScanView & scan, double length)
  {
    points_.clear();
    const std::size_t skip = static_cast<std::size_t>(std::max(1, p.beam_skip));
    for (std::size_t i = 0; i < scan.ranges.size(); i += skip) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
      if (r > length + 1.0) continue;
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_inc;
      const Point pt{r * std::cos(a), r * std::sin(a)};
      const double bx = pt.x + p.lidar_to_rear_axle;
      const bool own_body = bx >= -rearOverhang() && bx <= frontLength() &&
        std::abs(pt.y) <= p.car_half_width;
      if (!own_body) points_.push_back(pt);
    }
  }

  // Short-term memory. Keep track of how the car has moved (from the speed
  // and steering we commanded: no odometry), and keep the nearby dots of the
  // last memory_time seconds in that fixed frame.
  void remember(double dt)
  {
    const double ds = speed_estimate_ * dt;
    odom_th_ += ds * std::tan(last_cmd_steer_) / std::max(0.01, p.wheelbase);
    odom_x_ += ds * std::cos(odom_th_);
    odom_y_ += ds * std::sin(odom_th_);
    clock_ += dt;
    while (!memory_.empty() && clock_ - memory_.front().t > p.memory_time) memory_.pop_front();
    const double c = std::cos(odom_th_);
    const double s = std::sin(odom_th_);
    const double lx = odom_x_ + p.lidar_to_rear_axle * c;
    const double ly = odom_y_ + p.lidar_to_rear_axle * s;
    const double r2 = p.memory_radius * p.memory_radius;
    for (std::size_t i = 0; i < points_.size(); i += 2) {  // every other dot is plenty
      const Point & pt = points_[i];
      const double bx = pt.x + p.lidar_to_rear_axle;  // from the rear axle
      if (bx * bx + pt.y * pt.y > r2) continue;
      memory_.push_back({lx + c * pt.x - s * pt.y, ly + s * pt.x + c * pt.y, clock_});
    }
  }

  // THREAD mode: try every pair of steering angles (one for the first
  // thread_split m, another for the rest) and pick the path that gets through
  // while staying farthest from everything.
  Thread threadPlan(double aim)
  {
    const double m = p.max_steering;
    const double fractions[] = {0.0, 0.08, 0.2, 0.4, 0.7, 1.0};
    std::vector<double> angles;
    for (const double f : fractions) {
      angles.push_back(f * m);
      if (f > 0.0) angles.push_back(-f * m);
    }
    // Only dots a THREAD ghost could reach matter: keep those (much faster).
    near_.clear();
    const double reach = p.thread_horizon + frontLength() + p.car_half_width + p.thread_clear_cap + 0.2;
    for (const Point & pt : points_) {
      const double bx = pt.x + p.lidar_to_rear_axle;
      if (bx * bx + pt.y * pt.y <= reach * reach) near_.push_back(pt);
    }
    Thread best;
    double best_score = -std::numeric_limits<double>::infinity();
    Thread furthest;  // fallback if nothing gets all the way through
    for (const double s1 : angles) {
      for (const double s2 : angles) {
        const Thread t = threadGhost(s1, s2);
        if (t.free > furthest.free + 1e-9) furthest = t;
        if (t.free < p.thread_horizon - 1e-6) continue;  // touches something: out
        const double progress = std::hypot(t.end_x + p.lidar_to_rear_axle, t.end_y) /
                                std::max(0.1, p.thread_horizon);
        double score =
          p.thread_clear_weight * std::min(t.clearance, p.thread_clear_cap)
          + p.thread_progress_weight * progress
          - p.thread_align_weight * std::abs(t.end_heading - aim)
          - p.thread_commit_weight * (std::abs(s1 - steer_) + 0.5 * std::abs(s2 - thread_steer2_)) /
              std::max(0.05, m);
        if (failed_side_ != 0.0 && t.end_y * failed_side_ > 0.05) score -= p.thread_failed_weight;
        if (score > best_score) { best_score = score; best = t; }
      }
    }
    const Thread pick = std::isfinite(best_score) ? best : furthest;
    thread_steer2_ = pick.steer2;
    chosen_path_ = threadPath(pick.steer1, pick.steer2);
    return pick;
  }

  Path threadPath(double s1, double s2) const
  {
    const double wheelbase = std::max(0.01, p.wheelbase);
    const double swing_time = p.reaction_time +
      0.5 * std::abs(s1 - steer_) / std::max(0.1, p.steer_rate);
    const Piece lead{std::tan(steer_) / wheelbase, std::max(0.0, last_speed_) * swing_time};
    return {{lead, {std::tan(s1) / wheelbase, p.thread_split},
             {std::tan(s2) / wheelbase, p.thread_horizon + 1.0}, {0.0, 0.0}}};
  }

  // Drive one THREAD ghost: how far until contact (body + thread_margin), and
  // the closest it comes to any point on the way.
  Thread threadGhost(double s1, double s2) const
  {
    ++ghost_count_;
    Thread t;
    t.steer1 = s1;
    t.steer2 = s2;
    const Path path = threadPath(s1, s2);
    const double step = std::max(0.02, p.thread_step);
    const int steps = static_cast<int>(std::ceil(p.thread_horizon / step));
    const double rear = rearOverhang();
    const double front = frontLength();
    const double look = std::max(rear, front) + p.car_half_width + p.thread_clear_cap + 0.05;
    double clearance = p.thread_clear_cap;
    Pose g{-p.lidar_to_rear_axle, 0.0, 1.0, 0.0};
    for (int k = 1; k <= steps; ++k) {
      const double distance = std::min(k * step, p.thread_horizon);
      g = poseOn(path, distance);
      const double margin = std::min(p.thread_margin, p.margin_spread * distance);
      for (const Point & pt : near_) {
        const double rx = pt.x - g.x;
        const double ry = pt.y - g.y;
        if (std::abs(rx) > look || std::abs(ry) > look) continue;
        const double bx = g.c * rx + g.s * ry;
        const double by = std::abs(-g.s * rx + g.c * ry);
        const double dx = std::max({-rear - bx, 0.0, bx - front});
        const double dy = std::max(by - p.car_half_width, 0.0);
        if (dx <= 0.0 && dy <= margin) {  // contact
          t.free = (k - 1) * step;
          t.clearance = clearance;
          t.end_x = g.x; t.end_y = g.y; t.end_heading = std::atan2(g.s, g.c);
          return t;
        }
        if (k > 1) clearance = std::min(clearance, std::hypot(dx, dy));
      }
    }
    t.free = p.thread_horizon;
    t.clearance = clearance;
    t.end_x = g.x; t.end_y = g.y; t.end_heading = std::atan2(g.s, g.c);
    return t;
  }

  double rearOverhang() const { return p.lidar_to_rear - p.lidar_to_rear_axle; }   // axle -> rear bumper
  double frontLength() const { return p.lidar_to_rear_axle + p.lidar_to_front; }   // axle -> front bumper

  Result drive(const Ghost & g, double length, double comfort_margin, double near) const
  {
    return {g, ghostFree(g, length, comfort_margin, near)};
  }

  // Drive one ghost and report how far it gets before
  //   .hard      : its body (+ hard margin) touches a point
  //   .comfort   : a point comes inside its comfort margin
  //   .clearance : and the tightest side clearance on the near part of the way
  // A point that is ALREADY alongside the car (inside the comfort band right
  // now) only counts on real contact: we're passing it, not heading for it.
  Free ghostFree(const Ghost & ghost, double length, double comfort_margin, double near) const
  {
    ++ghost_count_;
    const double roomy = p.side_margin + p.wall_turn_extra_margin;
    double clearance = roomy;  // tightest side clearance within `near` meters
    const double step = std::max(0.02, p.ghost_step);
    const int steps = static_cast<int>(std::ceil(length / step));
    const double rear = rearOverhang();
    const double front = frontLength() + p.front_margin;
    const double reach = std::hypot(std::max(rear, front), p.car_half_width + std::max(comfort_margin, roomy)) + 1e-6;
    const double reach2 = reach * reach;
    const Path path = pathFor(ghost);
    int comfort_hit = 0;  // step of the first comfort-margin contact (0 = none yet)
    for (int k = 1; k <= steps; ++k) {
      const double distance = k * step;
      const Pose g = poseOn(path, distance);
      const double hard = p.car_half_width + std::min(p.hard_margin, p.margin_spread * distance);
      const double comfort = p.car_half_width + std::min(comfort_margin, p.margin_spread * distance);
      for (const Point & pt : points_) {
        const double rx = pt.x - g.x;
        const double ry = pt.y - g.y;
        if (rx * rx + ry * ry > reach2) continue;
        const double bx = g.c * rx + g.s * ry;        // along the ghost's body
        if (bx < -rear || bx > front) continue;
        const double by = std::abs(-g.s * rx + g.c * ry);  // sideways from its centerline
        // Side clearance counts points beside the body, not ones just ahead of the bumper.
        if (distance <= near && bx <= front - p.front_margin - 0.10) {
          clearance = std::min(clearance, std::max(0.0, by - p.car_half_width));
        }
        if (by <= hard) {
          const double free = (k - 1) * step;
          return {free, comfort_hit > 0 ? (comfort_hit - 1) * step : free, clearance};
        }
        if (comfort_hit == 0 && by <= comfort) {
          const double bx0 = pt.x + p.lidar_to_rear_axle;
          const bool alongside = bx0 >= -rear && bx0 <= front &&
            std::abs(pt.y) <= p.car_half_width + comfort_margin;
          if (!alongside) comfort_hit = k;
        }
      }
    }
    return {length, comfort_hit > 0 ? (comfort_hit - 1) * step : length, clearance};
  }

  // How far (up to 0.5 m) the car can back up with this steering before its
  // body reaches a point the LiDAR can see. Points that are already alongside
  // the body only count if the body would actually swing into them.
  double reverseFree(double steer) const
  {
    // Check what we see now AND what we saw recently: by the time we back
    // up, things beside and behind the car are in the LiDAR's blind spot.
    std::vector<Point> check = points_;
    const double c = std::cos(odom_th_);
    const double s = std::sin(odom_th_);
    const double lx = odom_x_ + p.lidar_to_rear_axle * c;  // where the LiDAR is now
    const double ly = odom_y_ + p.lidar_to_rear_axle * s;
    for (const Seen & m : memory_) {
      const double dx = m.x - lx;
      const double dy = m.y - ly;
      check.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
    const double step = 0.05;
    const int steps = 10;  // looks up to 0.5 m behind
    const double rear = rearOverhang() + 0.05;
    const double front = frontLength();
    for (int k = 1; k <= steps; ++k) {
      const double distance = k * step;
      Pose g{-p.lidar_to_rear_axle - distance, 0.0, 1.0, 0.0};
      if (std::abs(steer) >= 1e-4) {
        const double radius = p.wheelbase / std::tan(std::abs(steer));
        const double phi = distance / radius;
        const double sign = steer > 0.0 ? 1.0 : -1.0;
        g = {-p.lidar_to_rear_axle - radius * std::sin(phi), sign * radius * (1.0 - std::cos(phi)),
             std::cos(-sign * phi), std::sin(-sign * phi)};
      }
      for (const Point & pt : check) {
        const double rx = pt.x - g.x;
        const double ry = pt.y - g.y;
        const double bx = g.c * rx + g.s * ry;
        if (bx < -rear || bx > front) continue;
        const double bx0 = pt.x + p.lidar_to_rear_axle;
        const bool alongside = bx0 >= -rearOverhang() && bx0 <= front;
        const double half = p.car_half_width + (alongside ? 0.0 : 0.03);
        if (std::abs(-g.s * rx + g.c * ry) <= half) return (k - 1) * step;
      }
    }
    return steps * step;
  }

  // Range toward angle_deg (0 = straight ahead, + = left), taken as the
  // median of the beams within +/- wall_beam_window_deg (Lab 3's noise fix).
  double medianRange(const ScanView & scan, double angle_deg) const
  {
    const int n = static_cast<int>(scan.ranges.size());
    const double position = (radians(angle_deg) - scan.angle_min) / scan.angle_inc;
    const int idx = std::clamp(static_cast<int>(std::lround(position)), 0, n - 1);
    const int half = std::max(0, static_cast<int>(
      std::lround(radians(p.wall_beam_window_deg) / scan.angle_inc)));
    std::vector<double> window;
    window.reserve(static_cast<std::size_t>(2 * half + 1));
    for (int i = std::max(0, idx - half); i <= std::min(n - 1, idx + half); ++i) {
      const double r = scan.ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(r) || r > scan.range_max) window.push_back(scan.range_max);
      else window.push_back(std::max(r, scan.range_min));
    }
    auto middle = window.begin() + static_cast<std::ptrdiff_t>(window.size() / 2);
    std::nth_element(window.begin(), middle, window.end());
    return *middle;
  }

  // The wall follower's opinion: the heading that would hold wall_distance.
  // Same two-beam math as Lab 3: beam b points straight at the wall side,
  // beam a is theta degrees further forward. Returns 0 (no opinion) and
  // trusted = false when the beams are not on one wall.
  double wallHeading(const ScanView & scan, bool & trusted, double & distance, double & alpha) const
  {
    const double theta = radians(std::clamp(p.wall_theta_deg, 10.0, 70.0));
    const double b = medianRange(scan, p.wall_side * 90.0);
    const double a = medianRange(scan, p.wall_side * (90.0 - degrees(theta)));
    alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
    distance = b * std::cos(alpha);
    trusted = std::abs(alpha) <= radians(p.wall_max_angle_deg);
    if (!trusted) return 0.0;

    const double projected = distance + p.wall_lookahead * std::sin(alpha);
    const double error = p.wall_distance - projected;    // + = too close
    const double h = -p.wall_side * p.wall_kp * error;   // too close -> steer away
    const bool toward_wall = p.wall_side * h > 0.0;
    const double limit = radians(toward_wall ? p.wall_approach_limit_deg : p.wall_away_limit_deg);
    return std::clamp(h, -limit, limit);
  }

  std::vector<Point> points_;
  std::vector<Result> fan_;   // every ghost of the current plan
  double steer_ = 0.0;        // smoothed steering command (rad, + = left)
  double lane_heading_ = 0.0; // which way the hallway runs, relative to the car (rad, + = left)
  bool thread_ = false;       // THREAD mode on
  int open_plans_ = 0;        // plans in a row with open space (to leave THREAD mode)
  double thread_steer2_ = 0.0;  // second steering angle of the last THREAD plan
  double failed_side_ = 0.0;  // +1 / -1: the side the last stuck THREAD attempt tried
  Path chosen_path_{};        // the path the car is following (for drawing)
  struct Seen { double x, y, t; };
  std::deque<Seen> memory_;   // recently seen nearby dots, in the dead-reckoned frame
  std::vector<Point> near_;   // dots within reach of a THREAD ghost
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_th_ = 0.0;  // rear axle, dead-reckoned
  double clock_ = 0.0;        // s since start
  double last_cmd_steer_ = 0.0;
  double last_speed_ = 0.0;   // speed we last commanded
  double speed_estimate_ = 0.0;  // our guess of the real speed (our "speedometer")
  bool reversing_ = false;
  double reversed_ = 0.0;
  double reverse_target_ = 0.0;
  int blocked_ = 0;
  int stuck_streak_ = 0;       // back-ups in a row without real progress
  double net_progress_ = 0.0;  // m gained since the last back-up began
  struct Step { double steer, distance; };
  std::deque<Step> trail_;     // what we commanded recently, oldest first
  double trail_length_ = 0.0;
  mutable int ghost_count_ = 0;  // ghosts driven in the current plan (for the timing report)
};
}  // namespace gap
// ==== CORE END ====================================================================

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ReactiveFollowGap : public rclcpp::Node
{
public:
  ReactiveFollowGap() : Node("reactive_node")
  {
    gap::Params & p = planner_.p;

    // Every decimal parameter, straight from the table in the core.
    for (const gap::ParamRef & ref : gap::doubleParams()) {
      p.*(ref.field) = declare_parameter<double>(ref.name, p.*(ref.field));
    }
    p.follow_wall = declare_parameter<bool>("follow_wall", p.follow_wall);
    setWallSide(declare_parameter<std::string>("wall_side", "right"));
    p.back_up = declare_parameter<bool>("back_up", p.back_up);
    p.use_thread = declare_parameter<bool>("use_thread", p.use_thread);
    p.thread_exit_plans = static_cast<int>(declare_parameter<int>("thread_exit_plans", p.thread_exit_plans));
    p.beam_skip = static_cast<int>(declare_parameter<int>("beam_skip", p.beam_skip));
    p.blocked_plans = static_cast<int>(declare_parameter<int>("blocked_plans", p.blocked_plans));
    // The simulator sends scans far faster than a real LiDAR; plan at this rate.
    plan_rate_hz_ = declare_parameter<double>("plan_rate_hz", 50.0);

    param_cb_ = add_on_set_parameters_callback(
      std::bind(&ReactiveFollowGap::onParams, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/gap_follow/markers", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ReactiveFollowGap::onScan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Ghost-car planner: body %.2f m wide, margins %.2f comfort / %.2f hard, headings +/-%.0f deg "
      "every %.0f deg, speed %.1f-%.1f m/s, back-up %s",
      2.0 * p.car_half_width, p.side_margin, p.hard_margin, p.heading_limit_deg,
      p.heading_step_deg, p.min_speed, p.max_speed, p.back_up ? "ON" : "OFF");
    if (p.follow_wall) {
      RCLCPP_INFO(get_logger(),
        "Wall advisor ON: %s wall at %.2f m (kp=%.2f, lookahead=%.2f m) -> %s laps",
        p.wall_side < 0.0 ? "RIGHT" : "LEFT", p.wall_distance, p.wall_kp, p.wall_lookahead,
        p.wall_side < 0.0 ? "clockwise" : "counter-clockwise");
    } else {
      RCLCPP_INFO(get_logger(), "Wall advisor OFF (ghost cars only)");
    }
  }

private:
  void setWallSide(const std::string & side)
  {
    if (side != "right" && side != "left") {
      RCLCPP_WARN(get_logger(), "wall_side must be \"right\" or \"left\"; using right");
    }
    planner_.p.wall_side = (side == "left") ? 1.0 : -1.0;
  }

  // Live tuning: `ros2 param set /reactive_node <name> <value>` while driving.
  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    gap::Params & p = planner_.p;
    for (const auto & param : params) {
      const std::string & n = param.get_name();
      if (n == "follow_wall" || n == "back_up" || n == "use_thread") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          (n == "follow_wall" ? p.follow_wall : n == "back_up" ? p.back_up : p.use_thread) =
            param.as_bool();
          RCLCPP_INFO(get_logger(), "Live update: %s = %s", n.c_str(),
            param.as_bool() ? "true" : "false");
        }
        continue;
      }
      if (n == "wall_side") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          setWallSide(param.as_string());
          RCLCPP_INFO(get_logger(), "Live update: wall_side = %s",
            p.wall_side < 0.0 ? "right" : "left");
        }
        continue;
      }
      double * target = (n == "plan_rate_hz") ? &plan_rate_hz_ : nullptr;
      for (const gap::ParamRef & ref : gap::doubleParams()) {
        if (n == ref.name) target = &(p.*(ref.field));
      }
      if (!target) continue;
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = n + " needs a decimal value, e.g. 2.0 instead of 2";
        return result;
      }
      *target = param.as_double();
      RCLCPP_INFO(get_logger(), "Live update: %s = %.3f", n.c_str(), *target);
    }
    return result;
  }

  static geometry_msgs::msg::Point point(double x, double y)
  {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    return pt;
  }

  visualization_msgs::msg::Marker lineMarker(const sensor_msgs::msg::LaserScan & scan, int id,
    int type, float r, float g, float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = scan.header;
    marker.ns = "gap_follow";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.25);
    return marker;
  }

  // Orange line: the chosen ghost's path. Cyan box: the ghost car where that
  // path ends. Green line: the heading the wall advisor asked for.
  void publishMarkers(const sensor_msgs::msg::LaserScan & scan, const gap::Output & out)
  {
    const gap::Params & p = planner_.p;
    visualization_msgs::msg::MarkerArray markers;

    auto path = lineMarker(scan, 0, visualization_msgs::msg::Marker::LINE_STRIP, 1.0f, 0.6f, 0.0f);
    const double length = std::max(0.3, out.free);
    const double front = p.lidar_to_rear_axle + p.lidar_to_front;
    gap::Pose last{-p.lidar_to_rear_axle, 0.0, 1.0, 0.0};
    for (double d = 0.0; d <= length + 1e-6; d += 0.1) {
      last = planner_.plannedPose(d);
      path.points.push_back(point(last.x + front * last.c, last.y + front * last.s));
    }
    markers.markers.push_back(path);

    auto box = lineMarker(scan, 1, visualization_msgs::msg::Marker::LINE_STRIP, 0.0f, 1.0f, 1.0f);
    const double rear = -(p.lidar_to_rear - p.lidar_to_rear_axle);
    const double w = p.car_half_width;
    const double corners[5][2] = {{front, w}, {front, -w}, {rear, -w}, {rear, w}, {front, w}};
    for (const auto & corner : corners) {
      box.points.push_back(point(last.x + corner[0] * last.c - corner[1] * last.s,
                                 last.y + corner[0] * last.s + corner[1] * last.c));
    }
    markers.markers.push_back(box);

    auto wish = lineMarker(scan, 2, visualization_msgs::msg::Marker::LINE_STRIP, 0.0f, 1.0f, 0.0f);
    wish.points.push_back(point(0.0, 0.0));
    wish.points.push_back(point(std::cos(out.preferred), std::sin(out.preferred)));
    if (!p.follow_wall) wish.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(wish);

    marker_pub_->publish(markers);
  }

  void onScan(const sensor_msgs::msg::LaserScan & scan)
  {
    if (scan.ranges.empty() || !(scan.angle_increment > 0.0)) return;

    // Plan at plan_rate_hz; the last command stays in effect in between.
    const rclcpp::Time stamp(scan.header.stamp);
    double dt = 1.0 / std::max(1.0, plan_rate_hz_);
    if (has_last_plan_) {
      const double since = (stamp - last_plan_).seconds();
      if (since >= 0.0 && since < 0.9 * dt) return;
      if (since > 0.0 && since < 0.5) dt = since;
    }
    last_plan_ = stamp;
    has_last_plan_ = true;

    const gap::ScanView view{scan.ranges, scan.angle_min, scan.angle_increment,
                             scan.range_min, scan.range_max};
    const double speed_before = planner_.lastSpeed();

    // Time the planning itself: scan in -> steering angle and speed out.
    // steady_clock is a stopwatch: it only counts forward, so it is the right
    // clock for measuring how long something took.
    const auto plan_start = std::chrono::steady_clock::now();
    const gap::Output out = planner_.plan(view, dt);
    const double plan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - plan_start).count();
    recordPlanTime(plan_ms);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header = scan.header;
    command.drive.steering_angle = static_cast<float>(out.steer);
    command.drive.speed = static_cast<float>(out.speed);
    drive_pub_->publish(command);

    if (out.started_reverse) {
      RCLCPP_WARN(get_logger(), "No way forward: stopping, then backing up %.2f m the way we came",
        out.reverse_target);
    }
    if (out.boxed_in) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Boxed in: no way forward%s", planner_.p.back_up ? " and no room to back up" : "");
    }
    if (out.reversing) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "BACKING UP | steer=%+.3f | speed=%.2f", out.steer, out.speed);
      return;
    }
    publishMarkers(scan, out);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "%s | wants %+.0f deg | heading=%+.0f deg | ghost gets %.2f m | wall=%.2f m | "
      "steer=%+.3f | speed=%.2f (was %.2f) | plan %.2f ms for %d ghosts "
      "(last %d plans: avg %.2f, worst %.2f; worst ever %.2f; budget %.0f ms)",
      out.mode, gap::degrees(out.preferred), gap::degrees(out.heading), out.free,
      out.wall_distance, out.steer, out.speed, speed_before, plan_ms, out.ghosts,
      kTimingWindow, window_avg_ms_, window_worst_ms_, worst_ever_ms_,
      1000.0 / std::max(1.0, plan_rate_hz_));
  }

  // Keep the average and worst planning time over each block of
  // kTimingWindow plans (about one second), plus the worst ever seen.
  void recordPlanTime(double ms)
  {
    sum_ms_ += ms;
    block_worst_ms_ = std::max(block_worst_ms_, ms);
    worst_ever_ms_ = std::max(worst_ever_ms_, ms);
    if (++block_count_ >= kTimingWindow) {
      window_avg_ms_ = sum_ms_ / block_count_;
      window_worst_ms_ = block_worst_ms_;
      sum_ms_ = 0.0;
      block_worst_ms_ = 0.0;
      block_count_ = 0;
    }
  }

  gap::Planner planner_;
  double plan_rate_hz_ = 50.0;
  rclcpp::Time last_plan_{0, 0, RCL_ROS_TIME};
  bool has_last_plan_ = false;

  // Planning-time bookkeeping (milliseconds)
  static constexpr int kTimingWindow = 50;
  double sum_ms_ = 0.0, block_worst_ms_ = 0.0;
  int block_count_ = 0;
  double window_avg_ms_ = 0.0, window_worst_ms_ = 0.0, worst_ever_ms_ = 0.0;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReactiveFollowGap>());
  rclcpp::shutdown();
  return 0;
}
