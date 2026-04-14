function [public_vars] = student_workspace(read_only_vars, public_vars)
%STUDENT_WORKSPACE Main student control loop with explicit navigation states.

if read_only_vars.counter == 1
    public_vars = init_workspace_defaults(read_only_vars, public_vars);
end
public_vars.global_step = read_only_vars.counter;

gnss_valid = isfield(read_only_vars, 'gnss_position') && ~isempty(read_only_vars.gnss_position) ...
    && all(isfinite(read_only_vars.gnss_position(1:2)));
lidar_valid = isfield(read_only_vars, 'lidar_distances') && ~isempty(read_only_vars.lidar_distances) ...
    && any(isfinite(read_only_vars.lidar_distances));

% 9. Update particle filter
if lidar_valid
    [public_vars.particles, public_vars.particle_weights, public_vars.pf_stats] = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Fuse estimates and evaluate localization quality
public_vars = update_estimated_pose(read_only_vars, public_vars, gnss_valid, lidar_valid);

% Sensor histories
public_vars.lidar_history = [public_vars.lidar_history; read_only_vars.lidar_distances];
if ~isempty(read_only_vars.gnss_position)
    public_vars.gnss_history = [public_vars.gnss_history; read_only_vars.gnss_position];
end
public_vars.environment_ambiguity = evaluate_environment_ambiguity(read_only_vars, public_vars);
public_vars = update_localization_commit_state(read_only_vars, public_vars, gnss_valid, lidar_valid);
public_vars = update_localization_phase(public_vars);

public_vars = run_navigation_state_machine(read_only_vars, public_vars, gnss_valid, lidar_valid);
public_vars = update_localization_phase(public_vars);
end

function public_vars = init_workspace_defaults(read_only_vars, public_vars)
public_vars.lidar_history = [];
public_vars.gnss_history = [];
public_vars.debug.last_transition_reason = "init";
public_vars.debug.last_track_event = "none";
public_vars.debug.force_replan_reason = "none";
public_vars.debug.force_relocalize_reason = "none";

if ~isfield(public_vars, 'path_planner_mode') || isempty(public_vars.path_planner_mode)
    public_vars.path_planner_mode = 'astar';
end
if ~isfield(public_vars, 'force_grid_planner')
    public_vars.force_grid_planner = false;
end
if ~isfield(public_vars, 'replan_path')
    public_vars.replan_path = true;
end
if ~isfield(public_vars, 'path_smoothing_mode') || isempty(public_vars.path_smoothing_mode)
    public_vars.path_smoothing_mode = 'chaikin';
end
if ~isfield(public_vars, 'path_clearance_m') || ~isfinite(public_vars.path_clearance_m)
    public_vars.path_clearance_m = 0.25;
end
if ~isfield(public_vars, 'planner_clearance_m') || ~isfinite(public_vars.planner_clearance_m)
    public_vars.planner_clearance_m = public_vars.path_clearance_m;
end
if ~isfield(public_vars, 'tracking_clearance_m') || ~isfinite(public_vars.tracking_clearance_m)
    public_vars.tracking_clearance_m = public_vars.path_clearance_m;
end
if ~isfield(public_vars, 'path_resample_ds_m') || ~isfinite(public_vars.path_resample_ds_m)
    public_vars.path_resample_ds_m = 0.10;
end
if ~isfield(public_vars, 'path_refine_iters') || ~isfinite(public_vars.path_refine_iters)
    public_vars.path_refine_iters = 2;
end
if ~isfield(public_vars, 'smooth_iter_beta') || ~isfinite(public_vars.smooth_iter_beta)
    public_vars.smooth_iter_beta = 0.64;
end
if ~isfield(public_vars, 'smooth_chaikin_iters') || ~isfinite(public_vars.smooth_chaikin_iters)
    public_vars.smooth_chaikin_iters = 4;
end
if ~isfield(public_vars, 'localization_mode') || isempty(public_vars.localization_mode)
    public_vars.localization_mode = 'fusion';
end

public_vars.nav_state = "globalize";
public_vars.localization_phase = "globalize";
public_vars.nav_state_step = 0;
public_vars.path = [];
public_vars.raw_path = [];
public_vars.smoothed_path = [];
public_vars.global_path = [];
public_vars.display_path = [];
public_vars.path_revision = 0;
public_vars.path_length_m = 0;
public_vars.path_num_points = 0;
public_vars.path_history = {};
public_vars.path_anchor_pose = [nan, nan, nan];
public_vars.path_quality = struct('score', nan, 'min_clearance_m', nan, 'max_turn_deg', nan, ...
    'turn_density', nan, 'length_ratio', nan, 'planner_clearance_m', public_vars.planner_clearance_m, ...
    'tracking_clearance_m', public_vars.tracking_clearance_m, 'mode', string(public_vars.path_smoothing_mode));
public_vars.replan_distance_m = 0.9;
public_vars.replan_heading_rad = 55 * pi / 180;
public_vars.track_relocalize_grace_steps = 40;
public_vars.track_replan_grace_steps = 12;
public_vars.path_entry.max_steps = 80;
public_vars.path_entry.min_steps = 4;
public_vars.path_entry.handoff_steps = 24;
public_vars.path_entry.merge_path_dist_m = 0.18;
public_vars.path_entry.merge_heading_rad = 18 * pi / 180;
public_vars.path_entry.merge_path_dist_loose_m = 0.10;
public_vars.path_entry.merge_heading_loose_rad = 28 * pi / 180;
public_vars.path_entry.progress_dist_m = 0.12;
public_vars.path_entry.stuck_steps = 20;
public_vars.path_entry.stall_progress_eps_m = 0.03;
public_vars.path_entry.stall_heading_rad = 28 * pi / 180;
public_vars.path_entry.fail_path_dist_m = 0.60;
public_vars.path_entry.fail_heading_rad = 45 * pi / 180;
public_vars.path_entry.fail_progress_m = 0.08;
public_vars.path_entry.start_xy = [nan, nan];
public_vars.path_entry.best_progress_m = 0;
public_vars.path_entry.stall_counter = 0;
public_vars.path_entry.last_path_dist_m = inf;
public_vars.path_entry.last_heading_error_rad = inf;
public_vars.localize.stable_steps_required = 18;
public_vars.localize.stable_counter = 0;
public_vars.localize.min_steps = 180;
public_vars.localize.max_steps = 260;
public_vars.localize.hard_max_steps = 520;
public_vars.localize.pf_cluster_good = 0.35;
public_vars.localize.pf_cluster_ok = 0.55;
public_vars.localize.pf_cluster_plan = 0.70;
public_vars.localize.pf_cluster_force_plan = 0.24;
public_vars.localize.pf_dominant_mass_good = 0.72;
public_vars.localize.pf_dominant_mass_ok = 0.58;
public_vars.localize.pf_dominant_mass_plan = 0.70;
public_vars.localize.pf_dominant_mass_force_plan = 0.96;
public_vars.localize.pf_top_ratio_finish_ok = 1.08;
public_vars.localize.pf_top_ratio_finish_force = 1.12;
public_vars.localize.pf_top_ratio_single_ok = 1.10;
public_vars.localize.pf_top_ratio_single_force = 1.16;
public_vars.localize.finish_clear_steps_min = 2;
public_vars.localize.late_recovery_clear_steps = 18;
public_vars.localize.late_recovery_scan_cost = 0.08;
public_vars.localize.late_recovery_pf_cluster_m = 0.22;
public_vars.localize.late_recovery_pf_mass = 0.96;
public_vars.localize.late_recovery_est_jump_m = 0.20;
public_vars.localize.late_recovery_pf_top_ratio = 1.12;
public_vars.localize.late_recovery_disagree_m = 0.55;
public_vars.localize.late_recovery_max_contradictions = 1;
public_vars.localize.late_recovery_max_no_progress = 2;
public_vars.localize.near_goal_finish_goal_dist_m = 0.95;
public_vars.localize.near_goal_finish_scan_cost = 0.08;
public_vars.localize.near_goal_finish_pf_cluster_m = 0.16;
public_vars.localize.near_goal_finish_pf_mass = 0.995;
public_vars.localize.near_goal_finish_pf_top_ratio = 1.001;
public_vars.localize.near_goal_finish_clear_steps = 4;
public_vars.localize.recovery_commit_cmd_distance_m = 3.20;
public_vars.localize.recovery_commit_trace_bbox_m = 1.00;
public_vars.localize.fast_recovery_commit_cmd_distance_m = 4.00;
public_vars.localize.fast_recovery_commit_trace_bbox_m = 2.20;
public_vars.localization_commit.state = "search";
public_vars.localization_commit.stable_steps = 0;
public_vars.localization_commit.unstable_steps = 0;
public_vars.localization_commit.confirm_required_steps = 10;
public_vars.localization_commit.drop_required_steps = 4;
public_vars.localization_commit.pf_cluster_confirm_m = 0.32;
public_vars.localization_commit.pf_cluster_commit_m = 0.20;
public_vars.localization_commit.pf_cluster_single_commit_m = 0.26;
public_vars.localization_commit.pf_mass_confirm = 0.80;
public_vars.localization_commit.pf_mass_commit = 0.95;
public_vars.localization_commit.pf_mass_single_commit = 0.92;
public_vars.localization_commit.pf_hyp_confirm_max = 2;
public_vars.localization_commit.pf_hyp_commit_max = 1;
public_vars.localization_commit.pf_top_ratio_confirm = 1.14;
public_vars.localization_commit.pf_top_ratio_commit = 1.28;
public_vars.localization_commit.pf_top_ratio_single_confirm = 1.12;
public_vars.localization_commit.pf_top_ratio_single_commit = 1.18;
public_vars.localization_commit.scan_cost_confirm = 0.18;
public_vars.localization_commit.scan_cost_commit = 0.10;
public_vars.localization_commit.scan_cost_single_commit = 0.14;
public_vars.localization_commit.hypothesis_age_confirm = 3;
public_vars.localization_commit.hypothesis_age_commit = 6;
public_vars.localization_commit.hypothesis_stability_confirm = 0.45;
public_vars.localization_commit.hypothesis_stability_commit = 0.60;
public_vars.localization_commit.single_commit_clear_steps = 4;
public_vars.localization_commit.single_commit_min_trace_bbox_m = 1.30;
public_vars.localization_commit.h2_mass_ratio_confirm_max = 0.82;
public_vars.localization_commit.h2_mass_ratio_commit_max = 0.68;
public_vars.localization_commit.h2_scan_gap_confirm_min = 0.06;
public_vars.localization_commit.h2_scan_gap_commit_min = 0.10;
public_vars.localization_commit.h2_stability_gap_commit_min = 0.08;
public_vars.localization_commit.h2_pose_sep_commit_min = 1.20;
public_vars.localization_commit.commit_pose = [nan, nan, nan];
public_vars.localization_commit.commit_score = -inf;
public_vars.localization_commit.enter_step = 1;
public_vars.localize.kf_var_good = 0.18;
public_vars.localize.kf_var_ok = 0.35;
public_vars.localize.disagree_good = 0.55;
public_vars.localize.disagree_ok = 1.00;
public_vars.localize.disagree_force_plan = 0.70;
public_vars.localize.scan_cost_good = 0.16;
public_vars.localize.scan_cost_ok = 0.22;
public_vars.localize.scan_cost_force_plan = 0.14;
public_vars.localize.motion_step = 0;
public_vars.localize.forward_speed = 0.14;
public_vars.localize.turn_rate = 0.34;
public_vars.localize.turn_rate_blocked = 0.44;
public_vars.localize.wall_side = "none";
public_vars.localize.wall_align_counter = 0;
public_vars.localize.min_cmd_distance = 1.4;
public_vars.localize.min_cmd_turn = 1.45 * pi;
public_vars.localize.force_plan_cmd_distance = 2.6;
public_vars.localize.force_plan_stable_steps = 10;
public_vars.localize.cmd_distance_accum = 0;
public_vars.localize.cmd_turn_accum = 0;
public_vars.localize.reseed_period = 60;
public_vars.localize.reseed_period_min = 18;
public_vars.localize.reseed_period_max = 90;
public_vars.localize.allow_global_reseed = true;
public_vars.localize.reseed_stride = 1;
public_vars.localize.reseed_orientations = 16;
public_vars.localize.reseed_count = 240;
public_vars.localize.reseed_attempt_count = 0;
public_vars.localize.reseed_success_count = 0;
public_vars.localize.last_reseed_step = -inf;
public_vars.localize.last_reseed_score = 0;
public_vars.localize.reseed_crisis_trigger = 0.52;
public_vars.localize.reseed_fraction_min = 0.40;
public_vars.localize.reseed_fraction_max = 0.85;
public_vars.localize.post_reseed_hold_steps = 72;
public_vars.localize.post_reseed_hold_counter = 0;
public_vars.localize.info_gain_recovery_steps = 80;
public_vars.localize.info_gain_recovery_counter = 0;
public_vars.localize.fast_exit_min_steps = 90;
public_vars.localize.fast_exit_pf_cluster = 0.42;
public_vars.localize.fast_exit_pf_mass = 0.78;
public_vars.localize.fast_exit_stable_steps = 8;
public_vars.localize.fast_exit_cmd_distance = 0.7;
public_vars.localize.fast_exit_cmd_turn = 0.5 * pi;
public_vars.localize.fast_recovery = false;
public_vars.localize.late_exit_min_steps = 40;
public_vars.localize.late_exit_pf_cluster = 0.22;
public_vars.localize.late_exit_pf_mass = 0.95;
public_vars.localize.late_exit_stable_steps = 8;
public_vars.localize.late_exit_scan_cost = 0.18;
public_vars.localize.recovery_exit_scan_cost = 0.02;
public_vars.localize.contradiction_counter = 0;
public_vars.localize.no_progress_counter = 0;
public_vars.localize.hard_reset_counter = 0;
public_vars.localize.commit_failure_count = 0;
public_vars.localize.commit_failure_extra_clear_steps = 3;
public_vars.localize.commit_failure_max_penalty = 4;
public_vars.localize.last_failed_commit_pose = [nan, nan, nan];
public_vars.localize.failed_commit_block_radius_m = 1.75;
public_vars.localize.contradiction_scan_cost = 0.24;
public_vars.localize.contradiction_disagree_m = 0.95;
public_vars.localize.contradiction_pf_cluster = 0.34;
public_vars.localize.contradiction_pf_mass = 0.84;
public_vars.localize.contradiction_pf_ratio = 1.10;
public_vars.localize.contradiction_scan_change = 0.12;
public_vars.localize.contradiction_steps = 8;
public_vars.localize.revoke_steps = 4;
public_vars.localize.hard_reset_steps = 14;
public_vars.localize.no_progress_min_steps = 24;
public_vars.localize.no_progress_cmd_distance = 0.75;
public_vars.localize.no_progress_scan_change = 0.12;
public_vars.localize.no_progress_clear_counter_max = 1;
public_vars.localize.no_progress_steps = 10;
public_vars.localize.trace_max_points = 60;
public_vars.localize.trace_loop_cmd_distance = 1.05;
public_vars.localize.trace_loop_bbox_diag_m = 0.80;
public_vars.localize.trace_loop_scan_change = 0.14;
public_vars.localize.trace_loop_steps = 8;
public_vars.localize.sideswap_jump_m = 1.60;
public_vars.localize.sideswap_scan_change_max = 0.12;
public_vars.localize.sideswap_pf_cluster = 0.36;
public_vars.localize.sideswap_pf_mass = 0.86;
public_vars.localize.sideswap_steps = 3;
public_vars.localize.sideswap_hold_steps = 18;
public_vars.localize.sideswap_immediate_jump_m = 4.50;
public_vars.localize.sideswap_immediate_max_steps = 40;
public_vars.localize.sideswap_immediate_pf_cluster_max = 1.80;
public_vars.localize.large_jump_block_m = 4.00;
public_vars.localize.large_jump_hold_steps = 120;
public_vars.localize.large_jump_immediate_steps = 70;
public_vars.localize.large_jump_hold_counter = 0;
public_vars.localize.min_informative_cmd_distance = 0.75;
public_vars.localize.min_informative_cmd_turn = 0.35 * pi;
public_vars.localize.min_informative_scan_change = 0.14;
public_vars.localize.min_informative_pf_ratio = 1.08;
public_vars.localize.min_informative_pf_mass = 0.74;
public_vars.localize.pose_trace = zeros(0, 2);
public_vars.localize.trace_bbox_diag_m = inf;
public_vars.localize.trace_last_state = "localize";
public_vars.localize.trace_loop_counter = 0;
public_vars.localize.trace_last_jump_m = 0;
public_vars.localize.sideswap_counter = 0;
public_vars.localize.sideswap_hold_counter = 0;
public_vars.localize.extended_commit_guard = false;
public_vars.localize.extended_commit_trace_bbox_m = 1.15;
public_vars.localize.extended_commit_cmd_distance_m = 1.20;
public_vars.localize.extended_commit_turn_rad = 2.40;
public_vars.localize.extended_commit_clear_steps = 6;
public_vars.startup_motion.sector_half_angle_rad = 18 * pi / 180;
public_vars.startup_motion.target_sector_min_angle_rad = 16 * pi / 180;
public_vars.startup_motion.closed_side_max_m = 1.35;
public_vars.startup_motion.closed_front_max_m = 2.2;
public_vars.startup_motion.closed_near_fraction = 0.55;
public_vars.startup_motion.open_beam_clip_m = 4.5;
public_vars.startup_motion.heading_gain_localize = 1.15;
public_vars.startup_motion.heading_gain_disambiguate = 1.35;
public_vars.startup_motion.center_gain_localize = 0.72;
public_vars.startup_motion.center_gain_disambiguate = 0.65;
public_vars.startup_motion.soft_center_gain_localize = 0.18;
public_vars.startup_motion.soft_center_gain_disambiguate = 0.22;
public_vars.startup_motion.w_max_localize = 0.66;
public_vars.startup_motion.w_max_disambiguate = 0.82;
public_vars.startup_motion.v_open_localize = 0.19;
public_vars.startup_motion.v_closed_localize = 0.16;
public_vars.startup_motion.v_open_disambiguate = 0.22;
public_vars.startup_motion.v_closed_disambiguate = 0.18;
public_vars.startup_motion.turn_in_place_angle_rad = 58 * pi / 180;
public_vars.startup_motion.slow_turn_angle_rad = 26 * pi / 180;
public_vars.startup_motion.closed_bias_gain = 0.85;
public_vars.startup_motion.closed_sweep_gain = 0.04;
public_vars.startup_motion.closed_sweep_period_steps = 14;
public_vars.startup_motion.closed_min_turn_rad_s = 0.20;
public_vars.startup_motion.closed_front_turn_only_m = 0.55;
public_vars.startup_motion.closed_front_hard_turn_m = 0.36;
public_vars.startup_motion.closed_front_arc_limit_m = 0.80;
public_vars.startup_motion.closed_front_preturn_m = 1.05;
public_vars.startup_motion.closed_preturn_heading_rad = 20 * pi / 180;
public_vars.startup_motion.closed_turn_in_place_w_localize = 0.52;
public_vars.startup_motion.closed_turn_in_place_w_disambiguate = 0.64;
public_vars.startup_motion.closed_arc_v_localize = 0.025;
public_vars.startup_motion.closed_arc_v_disambiguate = 0.035;
public_vars.startup_motion.best_sector_turn_only_rad = 14 * pi / 180;
public_vars.startup_motion.side_escape_dist_m = 0.50;
public_vars.startup_motion.side_escape_turn_localize = 0.70;
public_vars.startup_motion.side_escape_turn_disambiguate = 0.82;
public_vars.startup_motion.commit_heading_rad = nan;
public_vars.startup_motion.commit_score = -inf;
public_vars.startup_motion.commit_counter = 0;
public_vars.startup_motion.commit_steps_localize = 12;
public_vars.startup_motion.commit_steps_disambiguate = 9;
public_vars.startup_motion.commit_release_scan_change = 0.22;
public_vars.startup_motion.commit_release_better_score = 0.24;
public_vars.startup_motion.commit_forbid_reverse_rad = 85 * pi / 180;
public_vars.startup_motion.min_commit_front_clear = 0.95;
public_vars.startup_motion.heading_smooth_alpha_localize = 0.18;
public_vars.startup_motion.heading_smooth_alpha_disambiguate = 0.24;
public_vars.startup_motion.max_heading_step_localize = 11 * pi / 180;
public_vars.startup_motion.max_heading_step_disambiguate = 15 * pi / 180;
public_vars.startup_motion.info_gain_scan_change = 0.12;
public_vars.startup_motion.info_gain_heading_scale = 1.45;
public_vars.startup_motion.info_gain_wmax_scale = 1.55;
public_vars.startup_motion.info_gain_v_scale = 0.95;
public_vars.startup_motion.info_gain_turn_in_place_angle_rad = 30 * pi / 180;
public_vars.startup_motion.info_gain_slow_turn_angle_rad = 14 * pi / 180;
public_vars.startup_motion.filtered_heading_rad = nan;
public_vars.startup_motion.edge_margin_m = 1.20;
public_vars.startup_motion.edge_hard_margin_m = 0.55;
public_vars.startup_motion.edge_heading_gain = 0.85;
public_vars.startup_motion.edge_v_scale_min = 0.10;
public_vars.startup_motion.edge_turn_only_heading_rad = 36 * pi / 180;
public_vars.startup_motion.rotate_only_disagree_m = 1.80;
public_vars.startup_motion.rotate_only_pf_mass = 0.58;
public_vars.startup_motion.rotate_only_front_clear_m = 1.00;
public_vars.disambiguate.reentry_cooldown_steps = 120;
public_vars.disambiguate.cooldown_until_step = 0;
public_vars.startup_motion.last_mode = "unknown";
public_vars.startup_motion.last_best_heading = nan;
public_vars.startup_motion.last_left_open = inf;
public_vars.startup_motion.last_right_open = inf;
public_vars.startup_motion.last_front_open = inf;
public_vars.ambiguity.counter = 0;
public_vars.ambiguity.corridor_front_m = 1.35;
public_vars.ambiguity.side_similarity_m = 0.24;
public_vars.ambiguity.min_side_m = 0.45;
public_vars.ambiguity.max_side_m = 2.2;
public_vars.ambiguity.scan_change_small = 0.12;
public_vars.ambiguity.required_steps = 5;
public_vars.ambiguity.clear_required_steps = 4;
public_vars.ambiguity.clear_counter = 0;
public_vars.disambiguate.min_steps = 24;
public_vars.disambiguate.max_steps = 84;
public_vars.disambiguate.forward_speed = 0.16;
public_vars.disambiguate.turn_rate = 0.60;
public_vars.disambiguate.wall_side = "none";
public_vars.disambiguate.wall_align_counter = 0;
public_vars.disambiguate.stall_trigger_steps = 8;
public_vars.disambiguate.goal_est_trigger_m = 5.0;
public_vars.disambiguate.reach_radius_m = 0.45;
public_vars.disambiguate.min_goal_distance_m = 1.6;
public_vars.disambiguate.max_goal_distance_m = 5.5;
public_vars.disambiguate.path = [];
public_vars.disambiguate.goal_xy = [nan, nan];
public_vars.disambiguate.visited_goals = zeros(0, 2);
public_vars.disambiguate.visited_sectors = [];
public_vars.disambiguate.visited_signatures = strings(0, 1);
public_vars.disambiguate.goal_sector_id = nan;
public_vars.disambiguate.sector_width_rad = 45 * pi / 180;
public_vars.disambiguate.sector_revisit_penalty = 1.65;
public_vars.disambiguate.goal_switch_count = 0;
public_vars.disambiguate.clearance_bias_gain = 0.80;
public_vars.disambiguate.clearance_action_gain = 0.34;
public_vars.disambiguate.clearance_target_m = max(0.45, 1.7 * public_vars.tracking_clearance_m);
public_vars.disambiguate.goal_path_clearance_gain = 0.32;
public_vars.disambiguate.goal_path_turn_gain = 0.12;
public_vars.disambiguate.goal_length_penalty = 0.08;
public_vars.disambiguate.voronoi_ridge_gain = 0.95;
public_vars.disambiguate.voronoi_ridge_min_clearance_m = max(0.38, 1.35 * public_vars.tracking_clearance_m);
public_vars.disambiguate.voronoi_ridge_neighbor_tol_m = 0.04;
public_vars.verify.active = false;
public_vars.verify.goal_xy = [nan, nan];
public_vars.verify.start_xy = [nan, nan];
public_vars.verify.reach_radius_m = 0.60;
public_vars.verify.min_steps = 25;
public_vars.verify.max_steps = 240;
public_vars.verify.min_goal_distance_m = 1.8;
public_vars.verify.max_goal_distance_m = 4.0;
public_vars.verify.min_goal_separation_from_final_m = 1.0;
public_vars.verify.required_clear_steps = 5;
public_vars.verify.min_progress_m = 1.15;
public_vars.verify.max_goal_distance_increase_m = 0.55;
public_vars.verify.goal_distance_penalty = 0.28;
public_vars.verify.selecting = false;
public_vars.verify.navigation_allow_steps = 180;
public_vars.verify.navigation_allow_counter = 0;
public_vars.track.last_path_index = 1;
public_vars.track.best_path_index = 1;
public_vars.track.stall_steps = 0;
public_vars.track.max_stall_steps = 180;
public_vars.track.near_goal_loop_steps = 0;
public_vars.track.max_near_goal_loop_steps = 20;
public_vars.track.near_goal_relocalize_count = 0;
public_vars = apply_workspace_tuning_overrides(public_vars);
public_vars.track.near_goal_hard_reset_after = 2;
public_vars.track.last_goal_distance = inf;
public_vars.track.best_goal_distance = inf;
public_vars.track.entry_goal_distance = inf;
public_vars.track.last_progress_step = 0;
public_vars.track.goal_progress_reset_m = 0.03;
public_vars.track.last_remaining_path_m = inf;
public_vars.track.path_progress_reset_m = 0.06;
public_vars.track.max_path_distance_m = 0.95;
public_vars.track.false_goal_radius_m = 0.80;
public_vars.track.near_goal_loop_remaining_path_m = 1.05;
public_vars.track.min_progress_index_jump = 1;
public_vars.track.false_goal_caution_radius_m = 2.2;
public_vars.track.false_goal_relocalize_stall_steps = 10;
public_vars.track.false_goal_stop_stall_steps = 25;
public_vars.track.false_goal_deadlock_stall_steps = 18;
public_vars.track.false_goal_confirm_stall_steps = 6;
public_vars.track.false_goal_confirm_goal_radius_m = 0.9;
public_vars.track.false_goal_confirm_path_dist_m = 0.20;
public_vars.track.fresh_commit_false_goal_counter = 0;
public_vars.track.fresh_commit_false_goal_steps = 3;
public_vars.track.fresh_commit_false_goal_radius_m = 0.22;
public_vars.track.fresh_commit_goal_jump_dist_m = 2.5;
public_vars.track.fresh_commit_path_dist_m = 0.18;
public_vars.track.away_goal_counter = 0;
public_vars.track.away_goal_trigger_steps = 14;
public_vars.track.away_goal_margin_m = 0.25;
public_vars.track.away_goal_step_m = 0.02;
public_vars.track.away_goal_path_dist_m = 0.30;
public_vars.track.away_goal_min_goal_dist_m = 1.4;
public_vars.track.away_goal_hard_margin_m = 1.80;
public_vars.track.near_collision_front_m = 0.24;
public_vars.track.near_collision_side_m = 0.14;
public_vars.track.near_collision_replan_front_m = 0.34;
public_vars.track.near_collision_replan_side_m = 0.18;
public_vars.track.front_risk_counter = 0;
public_vars.track.side_risk_counter = 0;
public_vars.track.map_conflict_counter = 0;
public_vars.track.offpath_replan_counter = 0;
public_vars.track.tight_front_replan_counter = 0;
public_vars.track.front_risk_steps_replan = 8;
public_vars.track.side_risk_steps_replan = 8;
public_vars.track.front_risk_steps_relocalize = 16;
public_vars.track.side_risk_steps_relocalize = 12;
public_vars.track.map_conflict_steps_relocalize = 14;
public_vars.track.commit_steps = 120;
public_vars.track.commit_counter = 0;
public_vars.track.uncommitted_counter = 0;
public_vars.track.uncommitted_relocalize_steps = 24;
public_vars.track.scan_mismatch_cost = inf;
public_vars.track.scan_mismatch_counter = 0;
public_vars.track.scan_mismatch_warn = 0.18;
public_vars.track.scan_mismatch_trigger = 0.28;
public_vars.track.scan_mismatch_steps = 8;
public_vars.track.commit_watchdog_counter = 0;
public_vars.track.commit_watchdog_steps = 6;
public_vars.track.commit_watchdog_scan_cost = 0.22;
public_vars.track.commit_watchdog_path_dist_m = 0.14;
public_vars.track.commit_watchdog_goal_dist_m = 2.2;
public_vars.track.commit_watchdog_scan_ema = 0.18;
public_vars.track.commit_watchdog_path_trend_m = 0.015;
public_vars.track.hard_path_dist_counter = 0;
public_vars.track.hard_path_dist_steps = 8;
public_vars.track.hard_path_dist_m = 1.5;
public_vars.track.offpath_replan_dist_m = 0.55;
public_vars.track.offpath_replan_hard_dist_m = 0.80;
public_vars.track.offpath_replan_steps = 14;
public_vars.track.offpath_replan_goal_progress_m = 0.015;
public_vars.track.tight_front_replan_front_m = 0.42;
public_vars.track.tight_front_replan_path_dist_m = 0.18;
public_vars.track.tight_front_replan_steps = 6;
public_vars.track.replan_cooldown_counter = 0;
public_vars.track.replan_cooldown_steps = 45;
public_vars.track.away_goal_min_path_dist_replan_m = 0.22;
public_vars.track.away_goal_min_counter_replan = 3;
public_vars.wall_follow.desired_dist_m = 0.60;
public_vars.wall_follow.capture_dist_m = 0.95;
public_vars.wall_follow.release_dist_m = 1.25;
public_vars.wall_follow.front_clear_m = 0.85;
public_vars.wall_follow.k_dist = 1.6;
public_vars.wall_follow.w_max = 0.55;
public_vars.wall_follow.k_align = 1.5;
public_vars.wall_follow.align_tol_rad = 8 * pi / 180;
public_vars.wall_follow.align_release_rad = 5 * pi / 180;
public_vars.wall_follow.align_required_steps = 4;
public_vars.wall_follow.v_align = 0.03;
public_vars.wall_follow.v_localize = 0.18;
public_vars.wall_follow.v_disambiguate = 0.20;
public_vars.pose_jump_guard.max_xy_jump_m = 1.2;
public_vars.pose_jump_guard.max_theta_jump_rad = 70 * pi / 180;
public_vars.pose_jump_guard.accept_pf_mass = 0.92;
public_vars.pose_jump_guard.accept_pf_ratio = 1.8;
public_vars.pose_jump_guard.accept_scan_cost = 0.12;
public_vars.pose_jump_guard.blend_factor = 0.18;
public_vars.quality_trend.alpha = 0.18;
public_vars.quality_trend.scan_match_ema = nan;
public_vars.quality_trend.path_distance_ema = nan;
public_vars.quality_trend.pf_cluster_ema = nan;
public_vars.quality_trend.disagreement_ema = nan;
public_vars.quality_trend.scan_match_trend = 0;
public_vars.quality_trend.path_distance_trend = 0;
public_vars.quality_trend.pf_cluster_trend = 0;
public_vars.quality_trend.disagreement_trend = 0;

public_vars = init_particle_filter(read_only_vars, public_vars);
public_vars = init_kalman_filter(read_only_vars, public_vars);
end

function public_vars = apply_workspace_tuning_overrides(public_vars)
global PROJECT_TUNING_OVERRIDES;
if isempty(PROJECT_TUNING_OVERRIDES) || ~isstruct(PROJECT_TUNING_OVERRIDES)
    return;
end

if isfield(PROJECT_TUNING_OVERRIDES, 'public_vars') && isstruct(PROJECT_TUNING_OVERRIDES.public_vars)
    override_struct = PROJECT_TUNING_OVERRIDES.public_vars;
else
    override_struct = PROJECT_TUNING_OVERRIDES;
end
public_vars = merge_struct_recursive(public_vars, override_struct);
end

function base = merge_struct_recursive(base, override)
if isempty(override) || ~isstruct(override)
    return;
end

fields = fieldnames(override);
for i = 1:numel(fields)
    name = fields{i};
    value = override.(name);
    if isstruct(value) && isscalar(value) && isfield(base, name) && isstruct(base.(name)) && isscalar(base.(name))
        base.(name) = merge_struct_recursive(base.(name), value);
    else
        base.(name) = value;
    end
end
end

function public_vars = update_estimated_pose(read_only_vars, public_vars, gnss_valid, lidar_valid)
pf_pose = estimate_pose(public_vars);
pf_valid = ~isempty(pf_pose) && all(isfinite(pf_pose));
kf_valid = gnss_valid && isfield(public_vars, 'kf_enabled') && public_vars.kf_enabled ...
    && isfield(public_vars, 'mu') && ~isempty(public_vars.mu) ...
    && all(isfinite(public_vars.mu(:)));
localization_mode = "fusion";
candidate_pose = [nan, nan, nan];
fusion_mode_value = 'none';
fusion_weights_xy = [nan, nan];
fusion_weights_theta = [nan, nan];
fusion_pf_score = nan;
fusion_kf_score = nan;
if isfield(public_vars, 'localization_mode') && ~isempty(public_vars.localization_mode)
    localization_mode = lower(string(public_vars.localization_mode));
end

if localization_mode == "pf_only"
    kf_valid = false;
elseif localization_mode == "kf_only"
    pf_valid = false;
end

if kf_valid && pf_valid
    kf_pose = public_vars.mu(:)';
    [pf_var_xy, pf_var_th] = pf_uncertainty(public_vars);
    if isfield(public_vars, 'sigma') && ~isempty(public_vars.sigma) && all(isfinite(public_vars.sigma(:)))
        kf_var_xy = max([public_vars.sigma(1,1), public_vars.sigma(2,2)], 1e-6);
        kf_var_th = max(public_vars.sigma(3,3), 1e-6);
    else
        kf_var_xy = [1, 1];
        kf_var_th = 1;
    end
    d_xy = norm(kf_pose(1:2) - pf_pose(1:2));
    d_th = abs(wrap_to_pi_ws(kf_pose(3) - pf_pose(3)));
    [fusion_pf_score, fusion_kf_score, agree_soft] = compute_localizer_quality_scores(read_only_vars, public_vars, gnss_valid, lidar_valid, ...
        pf_pose, pf_var_xy, pf_var_th, kf_pose, kf_var_xy, kf_var_th, d_xy, d_th);
    pf_xy_var_eff = max(mean(pf_var_xy), 1e-6) / max(fusion_pf_score, 0.05);
    kf_xy_var_eff = max(mean(kf_var_xy), 1e-6) / max(fusion_kf_score, 0.05);
    pf_th_var_eff = max(pf_var_th, 1e-6) / max(fusion_pf_score, 0.05);
    kf_th_var_eff = max(kf_var_th, 1e-6) / max(fusion_kf_score, 0.05);
    dominant_kf = fusion_kf_score >= 1.80 * max(fusion_pf_score, 1e-6);
    dominant_pf = fusion_pf_score >= 1.80 * max(fusion_kf_score, 1e-6);

    if gnss_valid && dominant_kf && (d_xy > 0.90 || d_th > 65 * pi / 180 || agree_soft < 0.28)
        candidate_pose = kf_pose;
        fusion_mode_value = 'kf_gnss_lock';
    elseif dominant_pf && agree_soft < 0.22
        candidate_pose = pf_pose;
        fusion_mode_value = 'pf_selected';
    else
        [xy_fused, w_xy] = fuse_xy(kf_pose(1:2), [kf_xy_var_eff, kf_xy_var_eff], pf_pose(1:2), [pf_xy_var_eff, pf_xy_var_eff]);
        [th_fused, w_th] = fuse_theta(kf_pose(3), kf_th_var_eff, pf_pose(3), pf_th_var_eff);
        candidate_pose = [xy_fused, th_fused];
        fusion_weights_xy = w_xy;
        fusion_weights_theta = w_th;
        if gnss_valid
            fusion_mode_value = 'blend_quality_gnss';
        else
            fusion_mode_value = 'blend_quality';
        end
    end
elseif kf_valid
    candidate_pose = public_vars.mu(:)';
    fusion_mode_value = 'kf_only';
    fusion_kf_score = 1.0;
elseif pf_valid
    candidate_pose = pf_pose;
    fusion_mode_value = 'pf_only';
    fusion_pf_score = 1.0;
elseif ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose)
    candidate_pose = [nan, nan, nan];
    fusion_mode_value = 'none';
end

candidate_pose = apply_pose_jump_guard(read_only_vars, public_vars, candidate_pose, lidar_valid);
public_vars.estimated_pose = candidate_pose;
public_vars.fusion.mode = fusion_mode_value;
public_vars.fusion.weights_xy = fusion_weights_xy;
public_vars.fusion.weights_theta = fusion_weights_theta;
public_vars.fusion.pf_score = fusion_pf_score;
public_vars.fusion.kf_score = fusion_kf_score;

public_vars.localization_quality = evaluate_localization_quality(read_only_vars, public_vars, pf_valid, kf_valid, gnss_valid, lidar_valid);
public_vars = update_localization_quality_trends(public_vars);
end

function [pf_score, kf_score, agree_soft] = compute_localizer_quality_scores(read_only_vars, public_vars, gnss_valid, lidar_valid, ...
    pf_pose, pf_var_xy, pf_var_th, kf_pose, kf_var_xy, kf_var_th, d_xy, d_th)
pf_score = 0.15;
kf_score = 0.15;

pf_stats = getfield_with_default(public_vars, 'pf_stats', struct());
pf_mass = getfield_with_default(pf_stats, 'dominant_mass', 0);
pf_ratio = getfield_with_default(pf_stats, 'top_ratio', 1);
pf_hyp = getfield_with_default(pf_stats, 'hypothesis_count', nan);
pf_cluster = inf;
if isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
    center = median(public_vars.particles(:, 1:2), 1);
    pf_cluster = median(vecnorm(public_vars.particles(:, 1:2) - center, 2, 2));
end

scan_cost = inf;
if lidar_valid && isfield(read_only_vars, 'lidar_distances') && any(isfinite(read_only_vars.lidar_distances))
    pred_pf = compute_lidar_measurement(read_only_vars.map, pf_pose, read_only_vars.lidar_config);
    scan_cost = lidar_match_cost(pred_pf, read_only_vars.lidar_distances);
end

[front_open, left_open, right_open] = lidar_sector_minima(read_only_vars);
front_space_open = (~isfinite(front_open)) || front_open >= 1.20;
left_space_open = (~isfinite(left_open)) || left_open >= 0.90;
right_space_open = (~isfinite(right_open)) || right_open >= 0.90;
open_gnss_space = gnss_valid && front_space_open && left_space_open && right_space_open;

pf_score = pf_score ...
    + 0.90 * min(max(pf_mass, 0), 1) ...
    + 0.35 * min(max(pf_ratio - 1.0, 0), 1.2) ...
    + 0.20 * double(isfinite(pf_hyp) && pf_hyp <= 2) ...
    + 0.18 * double(isfinite(pf_hyp) && pf_hyp <= 1);
if isfinite(pf_cluster)
    pf_score = pf_score + 0.55 * max(0, 1 - pf_cluster / 0.55);
end
if isfinite(mean(pf_var_xy))
    pf_score = pf_score + 0.30 * max(0, 1 - mean(pf_var_xy) / 0.40);
end
if isfinite(scan_cost)
    pf_score = pf_score + 0.60 * max(0, 1 - scan_cost / 0.22);
end

kf_score = kf_score + 0.85 * double(gnss_valid);
if isfinite(mean(kf_var_xy))
    kf_score = kf_score + 0.85 * max(0, 1 - mean(kf_var_xy) / 0.35);
end
if isfinite(kf_var_th)
    kf_score = kf_score + 0.20 * max(0, 1 - kf_var_th / 0.30);
end

agree_soft = exp(-0.5 * (d_xy / 0.90)^2 - 0.5 * (d_th / (55 * pi / 180))^2);
pf_score = pf_score * (0.55 + 0.45 * agree_soft);
kf_score = kf_score * (0.60 + 0.40 * agree_soft);
if gnss_valid
    kf_score = 1.10 * kf_score;
end
if open_gnss_space
    kf_score = 1.18 * kf_score + 0.18;
    pf_score = 0.92 * pf_score;
end
end

function public_vars = update_localization_quality_trends(public_vars)
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
if ~isfield(public_vars, 'quality_trend') || isempty(public_vars.quality_trend)
    return;
end

q = public_vars.localization_quality;
qt = public_vars.quality_trend;
alpha = max(0.05, min(0.45, getfield_with_default(qt, 'alpha', 0.18)));
[qt.scan_match_ema, qt.scan_match_trend] = update_ema_and_trend(getfield_with_default(qt, 'scan_match_ema', nan), ...
    getfield_with_default(q, 'scan_match_cost', nan), alpha);
[qt.path_distance_ema, qt.path_distance_trend] = update_ema_and_trend(getfield_with_default(qt, 'path_distance_ema', nan), ...
    getfield_with_default(q, 'path_distance_m', nan), alpha);
[qt.pf_cluster_ema, qt.pf_cluster_trend] = update_ema_and_trend(getfield_with_default(qt, 'pf_cluster_ema', nan), ...
    getfield_with_default(q, 'pf_cluster_radius_m', nan), alpha);
[qt.disagreement_ema, qt.disagreement_trend] = update_ema_and_trend(getfield_with_default(qt, 'disagreement_ema', nan), ...
    getfield_with_default(q, 'disagreement_xy_m', nan), alpha);
public_vars.quality_trend = qt;

public_vars.localization_quality.scan_match_ema = qt.scan_match_ema;
public_vars.localization_quality.path_distance_ema = qt.path_distance_ema;
public_vars.localization_quality.pf_cluster_ema = qt.pf_cluster_ema;
public_vars.localization_quality.disagreement_ema = qt.disagreement_ema;
public_vars.localization_quality.scan_match_trend = qt.scan_match_trend;
public_vars.localization_quality.path_distance_trend = qt.path_distance_trend;
public_vars.localization_quality.pf_cluster_trend = qt.pf_cluster_trend;
public_vars.localization_quality.disagreement_trend = qt.disagreement_trend;
end

function public_vars = update_localization_phase(public_vars)
phase = "globalize";
nav_state = string(getfield_with_default(public_vars, 'nav_state', "globalize"));
commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
if nav_state == "track"
    phase = "track";
elseif nav_state == "disambiguate"
    phase = "disambiguate";
elseif commit_state == "committed" && any(nav_state == ["plan", "path_entry", "commit"])
    phase = "commit";
elseif commit_state == "confirm"
    phase = "confirm";
elseif any(nav_state == ["relocalize", "localize", "globalize"])
    phase = "globalize";
end
public_vars.localization_phase = phase;
if isfield(public_vars, 'localization_quality') && ~isempty(public_vars.localization_quality)
    public_vars.localization_quality.localization_phase = phase;
end
end

function [ema, trend] = update_ema_and_trend(prev_ema, sample, alpha)
ema = prev_ema;
trend = 0;
if ~isfinite(sample)
    return;
end
if ~isfinite(prev_ema)
    ema = sample;
    return;
end
ema = (1 - alpha) * prev_ema + alpha * sample;
trend = ema - prev_ema;
end

function pose = apply_pose_jump_guard(read_only_vars, public_vars, pose, lidar_valid)
if isempty(pose) || any(~isfinite(pose(1:3)))
    return;
end
if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:3)))
    return;
end
if ~isfield(public_vars, 'nav_state')
    return;
end
if ~any(strcmp(string(public_vars.nav_state), ["localize", "relocalize", "disambiguate"]))
    return;
end

prev_pose = public_vars.estimated_pose;
jump_xy = norm(pose(1:2) - prev_pose(1:2));
jump_th = abs(wrap_to_pi_ws(pose(3) - prev_pose(3)));
if jump_xy <= public_vars.pose_jump_guard.max_xy_jump_m ...
        && jump_th <= public_vars.pose_jump_guard.max_theta_jump_rad
    return;
end

strong_pf = false;
if isfield(public_vars, 'pf_stats') && ~isempty(public_vars.pf_stats)
    strong_pf = isfield(public_vars.pf_stats, 'dominant_mass') && isfinite(public_vars.pf_stats.dominant_mass) ...
        && public_vars.pf_stats.dominant_mass >= public_vars.pose_jump_guard.accept_pf_mass ...
        && isfield(public_vars.pf_stats, 'top_ratio') && isfinite(public_vars.pf_stats.top_ratio) ...
        && public_vars.pf_stats.top_ratio >= public_vars.pose_jump_guard.accept_pf_ratio;
end
scan_good = false;
if lidar_valid && isfield(read_only_vars, 'map') && ~isempty(read_only_vars.map) ...
        && isfield(read_only_vars, 'lidar_config') && ~isempty(read_only_vars.lidar_config) ...
        && isfield(read_only_vars, 'lidar_distances') && ~isempty(read_only_vars.lidar_distances)
    pred = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
    scan_cost = lidar_match_cost(pred, read_only_vars.lidar_distances);
    scan_good = isfinite(scan_cost) && scan_cost <= public_vars.pose_jump_guard.accept_scan_cost;
end
if strong_pf || scan_good
    return;
end

blend = public_vars.pose_jump_guard.blend_factor;
pose_xy = prev_pose(1:2) + blend * (pose(1:2) - prev_pose(1:2));
pose_th = prev_pose(3) + blend * wrap_to_pi_ws(pose(3) - prev_pose(3));
pose = [pose_xy, wrap_to_pi_ws(pose_th)];
end

function public_vars = run_navigation_state_machine(read_only_vars, public_vars, gnss_valid, lidar_valid)
public_vars.nav_state_step = public_vars.nav_state_step + 1;
nav_read_only = apply_active_verify_goal_ws(read_only_vars, public_vars);
if isfield(public_vars, 'localize') && isfield(public_vars.localize, 'post_reseed_hold_counter')
    public_vars.localize.post_reseed_hold_counter = max(0, public_vars.localize.post_reseed_hold_counter - 1);
end
if isfield(public_vars, 'localize') && isfield(public_vars.localize, 'large_jump_hold_counter')
    public_vars.localize.large_jump_hold_counter = max(0, public_vars.localize.large_jump_hold_counter - 1);
end
if isfield(public_vars, 'localize') && isfield(public_vars.localize, 'info_gain_recovery_counter')
    public_vars.localize.info_gain_recovery_counter = max(0, public_vars.localize.info_gain_recovery_counter - 1);
end
if isfield(public_vars, 'verify') && isfield(public_vars.verify, 'navigation_allow_counter')
    public_vars.verify.navigation_allow_counter = max(0, public_vars.verify.navigation_allow_counter - 1);
end
public_vars.debug.last_transition_reason = "stay";
public_vars.debug.last_track_event = "none";
public_vars.debug.force_replan_reason = "none";
public_vars.debug.force_relocalize_reason = "none";
nav_gate = localization_navigation_gate_ws(public_vars);

switch string(public_vars.nav_state)
    case {"localize", "globalize", "confirm", "commit"}
        commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
        if string(public_vars.nav_state) == "commit" && commit_state == "committed"
            if ~nav_gate.can_plan
                public_vars.nav_state = nav_gate.recovery_state;
                public_vars.nav_state_step = 0;
                public_vars.localize.fast_recovery = false;
                public_vars.localize.allow_global_reseed = true;
                public_vars.debug.last_transition_reason = "commit_gate_block";
                public_vars.motion_vector = [0, 0];
                return;
            end
            public_vars = prepare_verify_goal_ws(read_only_vars, public_vars);
            public_vars.replan_path = true;
            public_vars.nav_state_step = 0;
            if isfield(public_vars, 'verify') && public_vars.verify.active ...
                    && all(isfinite(public_vars.verify.goal_xy))
                public_vars.nav_state = "verify";
                public_vars.debug.last_transition_reason = "commit_to_verify";
            else
                public_vars.nav_state = "plan";
                public_vars.debug.last_transition_reason = "commit_to_plan";
            end
            public_vars.motion_vector = [0, 0];
            return;
        end
        if any(string(public_vars.nav_state) == ["localize", "globalize", "confirm"]) ...
                && commit_state == "committed" && nav_gate.can_track
            public_vars.nav_state = "commit";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "gate_to_commit";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if string(public_vars.nav_state) == "confirm" && nav_gate.can_plan && ~nav_gate.can_track
            public_vars = prepare_verify_goal_ws(read_only_vars, public_vars);
            if isfield(public_vars, 'verify') && getfield_with_default(public_vars.verify, 'active', false) ...
                    && all(isfinite(public_vars.verify.goal_xy))
                public_vars.replan_path = true;
                public_vars.nav_state = "verify";
                public_vars.nav_state_step = 0;
                public_vars.debug.last_transition_reason = "confirm_to_verify";
                public_vars.motion_vector = [0, 0];
                return;
            end
        end
        if string(public_vars.nav_state) == "confirm" && commit_state == "search"
            public_vars.nav_state = "globalize";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "confirm_to_globalize_drop";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if should_trigger_global_reseed(public_vars, lidar_valid)
            public_vars = global_lidar_reseed(nav_read_only, public_vars);
        end
        [public_vars, hard_reset, reset_reason] = update_localization_recovery_monitor(nav_read_only, public_vars, lidar_valid);
        if hard_reset
            public_vars = global_lidar_reseed(nav_read_only, public_vars);
            public_vars.localize.allow_global_reseed = true;
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = reset_reason;
            public_vars.motion_vector = [0, 0];
            return;
        end
        [public_vars, finish_localize] = should_finish_localize(public_vars, gnss_valid, lidar_valid);
        if finish_localize
            if commit_state == "committed"
                public_vars.nav_state = "commit";
                public_vars.debug.last_transition_reason = "localize_finish_to_commit";
            elseif commit_state == "confirm"
                public_vars.nav_state = "confirm";
                public_vars.debug.last_transition_reason = "localize_finish_to_confirm";
            elseif should_enter_disambiguate(public_vars)
                public_vars.nav_state = "disambiguate";
                public_vars.debug.last_transition_reason = "localize_finish_to_disambiguate";
            else
                public_vars.nav_state = "confirm";
                public_vars.debug.last_transition_reason = "localize_finish_to_confirm_fallback";
            end
            public_vars.nav_state_step = 0;
            public_vars.motion_vector = [0, 0];
            return;
        end
        if string(public_vars.nav_state) ~= "commit" ...
                && public_vars.nav_state_step >= max(25, ceil(0.15 * public_vars.localize.min_steps)) ...
                && should_enter_disambiguate(public_vars)
            public_vars.nav_state = "disambiguate";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "localize_to_disambiguate_ambiguity";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if any(string(public_vars.nav_state) == ["localize", "globalize"]) ...
                && any(commit_state == ["search", "confirm"]) ...
                && public_vars.nav_state_step >= max(120, ceil(0.55 * public_vars.localize.min_steps)) ...
                && public_vars.localize.cmd_turn_accum >= max(1.2, 0.35 * public_vars.localize.min_cmd_turn) ...
                && (public_vars.localize.cmd_distance_accum >= max(0.25, 0.12 * public_vars.localize.min_cmd_distance) ...
                    || commit_state == "confirm")
            public_vars.nav_state = "disambiguate";
            public_vars.nav_state_step = 0;
            if commit_state == "confirm"
                public_vars.debug.last_transition_reason = "confirm_to_disambiguate_probe";
            else
                public_vars.debug.last_transition_reason = "globalize_to_disambiguate_probe";
            end
            public_vars.motion_vector = [0, 0];
            return;
        end
        public_vars = localization_exploration_motion(nav_read_only, public_vars);

    case "verify"
        nav_gate = localization_navigation_gate_ws(public_vars);
        if ~nav_gate.can_plan
            public_vars.nav_state = nav_gate.recovery_state;
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = true;
            public_vars.debug.last_transition_reason = "verify_gate_block";
            public_vars.motion_vector = [0, 0];
            return;
        end
        public_vars.replan_path = true;
        public_vars.nav_state = "plan";
        public_vars.nav_state_step = 0;
        public_vars.debug.last_transition_reason = "verify_to_plan";
        public_vars.motion_vector = [0, 0];
        return;

    case "plan"
        nav_gate = localization_navigation_gate_ws(public_vars);
        if ~nav_gate.can_plan
            public_vars.nav_state = nav_gate.recovery_state;
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = true;
            public_vars.debug.last_transition_reason = "plan_gate_block";
            public_vars.motion_vector = [0, 0];
            return;
        end
        [public_vars.path, public_vars.raw_path] = plan_path(nav_read_only, public_vars);
        public_vars.path = bridge_path_from_pose_ws(public_vars.path, public_vars.estimated_pose);
        public_vars.smoothed_path = public_vars.path;
        public_vars.global_path = public_vars.path;
        public_vars.display_path = public_vars.path;
        public_vars.replan_path = false;
        if ~isempty(public_vars.path) && size(public_vars.path, 1) >= 2
            public_vars.path_revision = public_vars.path_revision + 1;
            public_vars.path_num_points = size(public_vars.path, 1);
            public_vars.path_length_m = sum(vecnorm(diff(public_vars.path(:, 1:2), 1, 1), 2, 2));
            public_vars.path_quality = evaluate_path_quality_ws(public_vars.path, public_vars.raw_path, nav_read_only, public_vars);
            public_vars.path_history{end + 1} = public_vars.path; %#ok<AGROW>
            public_vars.nav_state = "path_entry";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "plan_to_path_entry_path_ready";
            public_vars.path_anchor_pose = public_vars.estimated_pose;
            public_vars.skip_start_alignment = false;
            public_vars.track.last_path_index = 1;
            public_vars.track.best_path_index = 1;
            public_vars.track.stall_steps = 0;
            public_vars.track.near_goal_loop_steps = 0;
            public_vars.track.near_goal_relocalize_count = 0;
            public_vars.track.fresh_commit_false_goal_counter = 0;
            public_vars.track.last_goal_distance = norm(public_vars.estimated_pose(1:2) - nav_read_only.map.goal(1:2));
            public_vars.track.best_goal_distance = public_vars.track.last_goal_distance;
            public_vars.track.entry_goal_distance = public_vars.track.last_goal_distance;
            public_vars.track.last_remaining_path_m = estimate_remaining_path_length(public_vars.path, 1, public_vars.estimated_pose(1:2));
            public_vars.track.last_progress_step = 0;
            public_vars.track.away_goal_counter = 0;
            public_vars.track.replan_cooldown_counter = public_vars.track.replan_cooldown_steps;
            public_vars.track.commit_counter = public_vars.track.commit_steps;
            public_vars.track.scan_mismatch_cost = inf;
            public_vars.track.scan_mismatch_counter = 0;
            public_vars.track.map_conflict_counter = 0;
            public_vars.path_entry.start_xy = public_vars.estimated_pose(1:2);
            public_vars.path_entry.best_progress_m = 0;
            public_vars.path_entry.stall_counter = 0;
            public_vars.path_entry.last_path_dist_m = inf;
            public_vars.path_entry.last_heading_error_rad = inf;
            public_vars.motion_vector = [0, 0];
        else
            public_vars.nav_state = "relocalize";
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = true;
            public_vars.debug.last_transition_reason = "plan_to_relocalize_empty_path";
            public_vars.motion_vector = [0, 0];
        end

    case "path_entry"
        nav_gate = localization_navigation_gate_ws(public_vars);
        if ~(nav_gate.can_track || (nav_gate.can_plan && nav_gate.can_info_track))
            public_vars.nav_state = nav_gate.recovery_state;
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = true;
            public_vars.debug.last_transition_reason = "path_entry_gate_block";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if should_enter_safe_stop(public_vars)
            public_vars.nav_state = "relocalize";
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = false;
            public_vars.debug.last_transition_reason = "path_entry_to_relocalize_invalid_pose";
            public_vars.motion_vector = [0, 0];
            return;
        end

        public_vars = plan_motion(nav_read_only, public_vars);
        [public_vars, entry_ready, entry_failed] = update_path_entry_state(nav_read_only, public_vars);
        if entry_ready
            public_vars.nav_state = "track";
            public_vars.nav_state_step = 0;
            public_vars.track.last_path_index = max(1, public_vars.track.best_path_index);
            public_vars.track.best_path_index = max(1, public_vars.track.best_path_index);
            public_vars.track.stall_steps = 0;
            public_vars.track.last_progress_step = 0;
            public_vars.track.commit_counter = public_vars.track.commit_steps;
            public_vars.track.uncommitted_counter = 0;
            public_vars.track.fresh_commit_false_goal_counter = 0;
            public_vars.track.entry_goal_distance = norm(public_vars.estimated_pose(1:2) - nav_read_only.map.goal(1:2));
            public_vars.track.replan_cooldown_counter = public_vars.track.replan_cooldown_steps;
            public_vars.skip_start_alignment = true;
            public_vars.localize.commit_failure_count = max(0, ...
                getfield_with_default(public_vars.localize, 'commit_failure_count', 0) - 1);
            public_vars.debug.last_transition_reason = "path_entry_to_track_merged";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if entry_failed
            public_vars.nav_state = "plan";
            public_vars.nav_state_step = 0;
            public_vars.replan_path = true;
            public_vars.debug.last_transition_reason = "path_entry_to_plan_failed_merge";
            public_vars.motion_vector = [0, 0];
            return;
        end

    case "track"
        nav_gate = localization_navigation_gate_ws(public_vars);
        if ~(nav_gate.can_track || (nav_gate.can_plan && nav_gate.can_info_track))
            public_vars.nav_state = nav_gate.recovery_state;
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.allow_global_reseed = true;
            public_vars.debug.last_transition_reason = "track_gate_block";
            public_vars.motion_vector = [0, 0];
            return;
        end
        public_vars = update_track_progress(nav_read_only, public_vars);
        if isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter')
            public_vars.track.commit_counter = max(0, public_vars.track.commit_counter - 1);
        end
        if isfield(public_vars, 'verify') && public_vars.verify.active
            verify_goal_dist = norm(public_vars.estimated_pose(1:2) - public_vars.verify.goal_xy(:)');
            verify_progress = inf;
            if all(isfinite(public_vars.verify.start_xy))
                verify_progress = norm(public_vars.estimated_pose(1:2) - public_vars.verify.start_xy(:)');
            end
            verify_clear_steps = get_ambiguity_clear_counter_ws(public_vars);
            if public_vars.nav_state_step >= public_vars.verify.min_steps ...
                    && (verify_goal_dist <= public_vars.verify.reach_radius_m ...
                        || verify_progress >= public_vars.verify.min_progress_m) ...
                    && verify_clear_steps >= public_vars.verify.required_clear_steps ...
                    && is_localization_confident_for_plan(public_vars, true)
                public_vars.verify.active = false;
                public_vars.verify.goal_xy = [nan, nan];
                public_vars.verify.start_xy = [nan, nan];
                public_vars.nav_state = "plan";
                public_vars.nav_state_step = 0;
                public_vars.replan_path = true;
                public_vars.debug.last_transition_reason = "verify_complete_to_plan";
                public_vars.motion_vector = [0, 0];
                return;
            elseif public_vars.nav_state_step >= public_vars.verify.max_steps ...
                    && ~is_localization_confident_for_plan(public_vars, true)
                public_vars.verify.active = false;
                public_vars.verify.goal_xy = [nan, nan];
                public_vars.verify.start_xy = [nan, nan];
                public_vars.nav_state = "relocalize";
                public_vars.nav_state_step = 0;
                public_vars.debug.last_transition_reason = "verify_timeout_to_relocalize";
                public_vars.motion_vector = [0, 0];
                return;
            end
        end
        if should_enter_safe_stop(public_vars)
            public_vars.nav_state = "safe_stop";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "track_to_safe_stop_invalid_pose";
            public_vars.motion_vector = [0, 0];
            return;
        end

        if isfield(public_vars, 'track') && isfield(public_vars.track, 'emergency_relocalize') ...
                && public_vars.track.emergency_relocalize
            relocalize_reason = string(public_vars.debug.force_relocalize_reason);
            if isfield(public_vars, 'verify') && getfield_with_default(public_vars.verify, 'active', false) ...
                    && any(relocalize_reason == ["near_goal_loop", "false_goal_deadlock", ...
                        "false_goal_stall", "false_goal_offpath", "fresh_commit_false_goal", ...
                        "track_stall_false_goal", "false_goal_ambiguity", "false_goal_weak_uniqueness"])
                public_vars = abort_verify_goal_ws(public_vars);
                public_vars.nav_state = "plan";
                public_vars.nav_state_step = 0;
                public_vars.replan_path = true;
                public_vars.debug.last_transition_reason = "verify_abort_to_plan_false_goal";
                public_vars.motion_vector = [0, 0];
                return;
            end
            hard_false_goal_reset = is_hard_false_goal_reset_reason(relocalize_reason);
            globalize_reset = any(relocalize_reason == ["near_goal_loop", "false_goal_deadlock", ...
                "false_goal_stall", "false_goal_offpath", "false_goal_uncommitted", "track_uncommitted", "fresh_commit_false_goal", ...
                "track_stall_false_goal", "false_goal_ambiguity", "false_goal_weak_uniqueness"]);
            if globalize_reset
                public_vars.nav_state = "globalize";
            else
                public_vars.nav_state = "relocalize";
            end
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = relocalize_reason == "map_scan_conflict" || hard_false_goal_reset;
            public_vars.localize.stable_counter = 0;
            public_vars.localize.cmd_distance_accum = 0;
            public_vars.localize.cmd_turn_accum = 0;
            public_vars.localize.post_reseed_hold_counter = max( ...
                getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0), ...
                getfield_with_default(public_vars.localize, 'post_reseed_hold_steps', 72));
            public_vars.localize.info_gain_recovery_counter = max( ...
                getfield_with_default(public_vars.localize, 'info_gain_recovery_counter', 0), ...
                getfield_with_default(public_vars.localize, 'info_gain_recovery_steps', 80));
            public_vars.localize.commit_failure_count = min( ...
                getfield_with_default(public_vars.localize, 'commit_failure_count', 0) + 1, ...
                getfield_with_default(public_vars.localize, 'commit_failure_max_penalty', 4));
            if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 3 ...
                    && all(isfinite(public_vars.estimated_pose(1:3)))
                public_vars.localize.last_failed_commit_pose = public_vars.estimated_pose(1:3);
            end
            public_vars.localize.allow_global_reseed = hard_false_goal_reset;
            public_vars.localize.extended_commit_guard = true;
            public_vars.localize.wall_side = "none";
            public_vars.localize.wall_align_counter = 0;
            if hard_false_goal_reset
                public_vars.track.near_goal_relocalize_count = getfield_with_default(public_vars.track, 'near_goal_relocalize_count', 0) + 1; %#ok<GFLD>
                public_vars = revoke_localization_commit(public_vars);
                if lidar_valid
                    public_vars = global_lidar_reseed(read_only_vars, public_vars);
                    public_vars.localize.allow_global_reseed = true;
                end
                if public_vars.track.near_goal_relocalize_count >= public_vars.track.near_goal_hard_reset_after
                    public_vars.localize.stable_counter = 0;
                end
            end
            public_vars.debug.last_transition_reason = "track_to_relocalize_emergency";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if isfield(public_vars, 'track') && isfield(public_vars.track, 'emergency_replan') ...
                && public_vars.track.emergency_replan
            public_vars.nav_state = "plan";
            public_vars.nav_state_step = 0;
            public_vars.replan_path = true;
            public_vars.debug.last_transition_reason = "track_to_plan_emergency";
            public_vars.motion_vector = [0, 0];
            return;
        end

        if should_enter_disambiguate(public_vars)
            public_vars.nav_state = "disambiguate";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "track_to_disambiguate_ambiguity";
            public_vars.motion_vector = [0, 0];
            return;
        end

        if public_vars.nav_state_step > public_vars.track_relocalize_grace_steps ...
                && should_relocalize(public_vars)
            relocalize_reason = string(public_vars.debug.force_relocalize_reason);
            hard_false_goal_reset = is_hard_false_goal_reset_reason(relocalize_reason);
            globalize_reset = any(relocalize_reason == ["near_goal_loop", "false_goal_deadlock", ...
                "false_goal_stall", "false_goal_offpath", "false_goal_uncommitted", "track_uncommitted", "fresh_commit_false_goal", ...
                "track_stall_false_goal", "false_goal_ambiguity", "false_goal_weak_uniqueness"]);
            if globalize_reset
                public_vars.nav_state = "globalize";
            else
                public_vars.nav_state = "relocalize";
            end
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = hard_false_goal_reset;
            public_vars.localize.allow_global_reseed = hard_false_goal_reset;
            public_vars.localize.extended_commit_guard = true;
            public_vars.localize.stable_counter = 0;
            public_vars.localize.cmd_distance_accum = 0;
            public_vars.localize.cmd_turn_accum = 0;
            public_vars.localize.post_reseed_hold_counter = max( ...
                getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0), ...
                getfield_with_default(public_vars.localize, 'post_reseed_hold_steps', 72));
            public_vars.localize.info_gain_recovery_counter = max( ...
                getfield_with_default(public_vars.localize, 'info_gain_recovery_counter', 0), ...
                getfield_with_default(public_vars.localize, 'info_gain_recovery_steps', 80));
            public_vars.localize.commit_failure_count = min( ...
                getfield_with_default(public_vars.localize, 'commit_failure_count', 0) + 1, ...
                getfield_with_default(public_vars.localize, 'commit_failure_max_penalty', 4));
            if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 3 ...
                    && all(isfinite(public_vars.estimated_pose(1:3)))
                public_vars.localize.last_failed_commit_pose = public_vars.estimated_pose(1:3);
            end
            if hard_false_goal_reset
                public_vars.track.near_goal_relocalize_count = getfield_with_default(public_vars.track, 'near_goal_relocalize_count', 0) + 1; %#ok<GFLD>
                public_vars = revoke_localization_commit(public_vars);
                if lidar_valid
                    public_vars = global_lidar_reseed(read_only_vars, public_vars);
                    public_vars.localize.allow_global_reseed = true;
                end
                if public_vars.track.near_goal_relocalize_count >= public_vars.track.near_goal_hard_reset_after
                    public_vars.localize.stable_counter = 0;
                end
            end
            public_vars.debug.last_transition_reason = "track_to_relocalize";
            public_vars.motion_vector = [0, 0];
            return;
        end

        if public_vars.nav_state_step > public_vars.track_replan_grace_steps ...
                && should_replan(public_vars)
            if isfield(public_vars, 'verify') && getfield_with_default(public_vars.verify, 'active', false)
                public_vars = abort_verify_goal_ws(public_vars);
                public_vars.nav_state = "plan";
                public_vars.nav_state_step = 0;
                public_vars.replan_path = true;
                public_vars.debug.last_transition_reason = "verify_abort_to_plan_replan";
                public_vars.motion_vector = [0, 0];
                return;
            end
            public_vars.nav_state = "plan";
            public_vars.nav_state_step = 0;
            public_vars.replan_path = true;
            public_vars.debug.last_transition_reason = "track_to_plan";
            public_vars.motion_vector = [0, 0];
            return;
        end

        public_vars = plan_motion(read_only_vars, public_vars);

    case "relocalize"
        nav_gate = localization_navigation_gate_ws(public_vars);
        commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
        if commit_state == "committed" && nav_gate.can_track
            public_vars.nav_state = "commit";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "relocalize_gate_to_commit";
            public_vars.motion_vector = [0, 0];
            return;
        elseif commit_state == "confirm" && nav_gate.can_plan
            public_vars.nav_state = "confirm";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "relocalize_gate_to_confirm";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if should_trigger_global_reseed(public_vars, lidar_valid)
            public_vars = global_lidar_reseed(read_only_vars, public_vars);
        end
        [public_vars, hard_reset, reset_reason] = update_localization_recovery_monitor(read_only_vars, public_vars, lidar_valid);
        if hard_reset
            public_vars = global_lidar_reseed(read_only_vars, public_vars);
            public_vars.localize.allow_global_reseed = true;
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = reset_reason;
            public_vars.motion_vector = [0, 0];
            return;
        end
        [public_vars, finish_localize] = should_finish_localize(public_vars, gnss_valid, lidar_valid);
        min_finish_distance = getfield_with_default(public_vars.localize, 'recovery_commit_cmd_distance_m', 2.00);
        if getfield_with_default(public_vars.localize, 'fast_recovery', false)
            min_finish_distance = max(min_finish_distance, ...
                getfield_with_default(public_vars.localize, 'fast_recovery_commit_cmd_distance_m', 4.00));
        end
        if finish_localize ...
                && getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0) < min_finish_distance
            finish_localize = false;
        end
        if finish_localize
            commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
            if commit_state == "committed"
                public_vars.nav_state = "commit";
                public_vars.debug.last_transition_reason = "relocalize_finish_to_commit";
            elseif commit_state == "confirm"
                public_vars.nav_state = "confirm";
                public_vars.debug.last_transition_reason = "relocalize_finish_to_confirm";
            elseif should_enter_disambiguate(public_vars)
                public_vars.nav_state = "disambiguate";
                public_vars.debug.last_transition_reason = "relocalize_finish_to_disambiguate";
            else
                public_vars.nav_state = "confirm";
                public_vars.debug.last_transition_reason = "relocalize_finish_to_confirm_fallback";
            end
            public_vars.nav_state_step = 0;
            public_vars.motion_vector = [0, 0];
            return;
        end
        if public_vars.nav_state_step >= max(20, ceil(0.10 * public_vars.localize.min_steps)) ...
                && should_enter_disambiguate(public_vars)
            public_vars.nav_state = "disambiguate";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "relocalize_to_disambiguate_ambiguity";
            public_vars.motion_vector = [0, 0];
            return;
        end
        public_vars = localization_exploration_motion(read_only_vars, public_vars);

    case "disambiguate"
        nav_gate = localization_navigation_gate_ws(public_vars);
        commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
        if commit_state == "committed" && nav_gate.can_track
            public_vars.nav_state = "commit";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "disambiguate_gate_to_commit";
            public_vars.motion_vector = [0, 0];
            return;
        elseif commit_state == "confirm" && nav_gate.can_plan
            public_vars.nav_state = "confirm";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "disambiguate_gate_to_confirm";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if should_enter_safe_stop(public_vars)
            public_vars.nav_state = "safe_stop";
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "disambiguate_to_safe_stop_invalid_pose";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if should_trigger_global_reseed(public_vars, lidar_valid)
            public_vars = global_lidar_reseed(read_only_vars, public_vars);
        end
        [public_vars, hard_reset, reset_reason] = update_localization_recovery_monitor(read_only_vars, public_vars, lidar_valid);
        if hard_reset
            public_vars = global_lidar_reseed(read_only_vars, public_vars);
            public_vars.localize.allow_global_reseed = true;
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = reset_reason;
            public_vars.motion_vector = [0, 0];
            return;
        end
        [public_vars, finish_localize] = should_finish_localize(public_vars, gnss_valid, lidar_valid);
        min_finish_distance = getfield_with_default(public_vars.localize, 'recovery_commit_cmd_distance_m', 2.00);
        if getfield_with_default(public_vars.localize, 'fast_recovery', false)
            min_finish_distance = max(min_finish_distance, ...
                getfield_with_default(public_vars.localize, 'fast_recovery_commit_cmd_distance_m', 4.00));
        end
        if finish_localize ...
                && getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0) < min_finish_distance
            finish_localize = false;
        end
        if finish_localize
            commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
            if commit_state == "committed"
                public_vars.nav_state = "commit";
                public_vars.debug.last_transition_reason = "disambiguate_finish_to_commit";
            elseif commit_state == "confirm"
                public_vars.nav_state = "confirm";
                public_vars.debug.last_transition_reason = "disambiguate_finish_to_confirm";
            else
                public_vars.nav_state = "relocalize";
                public_vars.debug.last_transition_reason = "disambiguate_finish_to_relocalize";
            end
            public_vars.nav_state_step = 0;
            public_vars.motion_vector = [0, 0];
            return;
        end
        if public_vars.nav_state_step == 1 || mod(public_vars.nav_state_step, 14) == 0 ...
                || ~isfield(public_vars.disambiguate, 'path') ...
                || isempty(public_vars.disambiguate.path)
            public_vars = prepare_disambiguation_path(read_only_vars, public_vars);
        end
        if isfield(public_vars.disambiguate, 'path') && ~isempty(public_vars.disambiguate.path) ...
                && should_use_disambiguation_path(read_only_vars, public_vars)
            tmp_public = public_vars;
            tmp_public.path = public_vars.disambiguate.path;
            tmp_public.smoothed_path = public_vars.disambiguate.path;
            tmp_public.raw_path = public_vars.disambiguate.path;
            tmp_public = plan_motion(read_only_vars, tmp_public);
            public_vars.motion_vector = apply_disambiguate_motion_limits(read_only_vars, public_vars, tmp_public.motion_vector);
            if isfield(tmp_public, 'motion_debug')
                public_vars.motion_debug = tmp_public.motion_debug;
            end
            if isfield(public_vars.disambiguate, 'goal_xy') && all(isfinite(public_vars.disambiguate.goal_xy))
                if norm(public_vars.estimated_pose(1:2) - public_vars.disambiguate.goal_xy(:)') <= public_vars.disambiguate.reach_radius_m
                    public_vars.disambiguate.visited_goals(end + 1, :) = public_vars.disambiguate.goal_xy(:)'; %#ok<AGROW>
                    public_vars = register_disambiguation_sector(public_vars);
                    public_vars.disambiguate.path = [];
                    public_vars.disambiguate.goal_xy = [nan, nan];
                    public_vars.disambiguate.cooldown_until_step = public_vars.global_step + public_vars.disambiguate.reentry_cooldown_steps;
                    public_vars.nav_state = "relocalize";
                    public_vars.nav_state_step = 0;
                    public_vars.localize.fast_recovery = false;
                    public_vars.localize.stable_counter = 0;
                    public_vars.localize.extended_commit_guard = true;
                    public_vars.debug.last_transition_reason = "disambiguate_goal_reached_to_relocalize";
                    public_vars.motion_vector = [0, 0];
                    return;
                end
            end
        else
            public_vars = disambiguation_motion(read_only_vars, public_vars);
        end
        if public_vars.nav_state_step >= public_vars.disambiguate.min_steps ...
                && is_localization_confident_for_plan(public_vars, true)
            public_vars.nav_state = "plan";
            public_vars.replan_path = true;
            public_vars.nav_state_step = 0;
            public_vars.debug.last_transition_reason = "disambiguate_confident_to_plan";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if public_vars.nav_state_step >= public_vars.disambiguate.min_steps ...
                && ~should_enter_disambiguate(public_vars)
            public_vars = register_disambiguation_sector(public_vars);
            public_vars.nav_state = "relocalize";
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.stable_counter = 0;
            public_vars.localize.extended_commit_guard = true;
            public_vars.disambiguate.cooldown_until_step = public_vars.global_step + public_vars.disambiguate.reentry_cooldown_steps;
            public_vars.debug.last_transition_reason = "disambiguate_clear_to_relocalize";
            public_vars.motion_vector = [0, 0];
            return;
        end
        if public_vars.nav_state_step >= public_vars.disambiguate.max_steps
            public_vars = register_disambiguation_sector(public_vars);
            public_vars.nav_state = "relocalize";
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.localize.stable_counter = 0;
            public_vars.localize.extended_commit_guard = true;
            public_vars.disambiguate.cooldown_until_step = public_vars.global_step + public_vars.disambiguate.reentry_cooldown_steps;
            public_vars.debug.last_transition_reason = "disambiguate_timeout_to_relocalize";
            public_vars.motion_vector = [0, 0];
            return;
        end

    case "safe_stop"
        public_vars.motion_vector = [0, 0];
        if ~should_enter_safe_stop(public_vars)
            public_vars.nav_state = "relocalize";
            public_vars.nav_state_step = 0;
            public_vars.localize.fast_recovery = false;
            public_vars.debug.last_transition_reason = "safe_stop_to_relocalize_recovered";
        end

    otherwise
        public_vars.nav_state = "localize";
        public_vars.nav_state_step = 0;
        public_vars.debug.last_transition_reason = "fallback_to_localize";
        public_vars.motion_vector = [0, 0];
end
end

function [public_vars, entry_ready, entry_failed] = update_path_entry_state(read_only_vars, public_vars)
entry_ready = false;
entry_failed = false;

if ~isfield(public_vars, 'path') || isempty(public_vars.path) || size(public_vars.path, 1) < 2 ...
        || ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:3)))
    entry_failed = true;
    return;
end

pose = public_vars.estimated_pose;
[seg_idx, path_dist, heading_err] = path_entry_metrics(public_vars.path, pose);
public_vars.track.last_path_index = seg_idx;
public_vars.track.best_path_index = max(public_vars.track.best_path_index, seg_idx);
public_vars.path_entry.last_path_dist_m = path_dist;
public_vars.path_entry.last_heading_error_rad = heading_err;
align_active = isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'align_active') ...
    && logical(public_vars.motion_debug.align_active);

progress_m = 0;
if isfield(public_vars.path_entry, 'start_xy') && all(isfinite(public_vars.path_entry.start_xy))
    progress_m = norm(pose(1:2) - public_vars.path_entry.start_xy(:)');
end
public_vars.path_entry.best_progress_m = max(public_vars.path_entry.best_progress_m, progress_m);

merged = public_vars.nav_state_step >= public_vars.path_entry.min_steps ...
    && ((path_dist <= 0.15 && ~align_active) ...
        || (path_dist <= public_vars.path_entry.merge_path_dist_m ...
            && heading_err <= public_vars.path_entry.merge_heading_loose_rad) ...
        || (path_dist <= 0.24 && heading_err <= public_vars.path_entry.merge_heading_rad ...
            && progress_m >= public_vars.path_entry.progress_dist_m));
if ~merged && public_vars.nav_state_step >= public_vars.path_entry.handoff_steps ...
        && ~align_active && progress_m >= public_vars.path_entry.progress_dist_m
    merged = true;
end
if merged
    public_vars.path_entry.stall_counter = 0;
    entry_ready = true;
    return;
end

progressing = progress_m >= public_vars.path_entry.progress_dist_m ...
    || (isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'align_active') && public_vars.motion_debug.align_active);
stall_like = path_dist > public_vars.path_entry.merge_path_dist_m ...
    && heading_err > public_vars.path_entry.stall_heading_rad ...
    && progress_m <= public_vars.path_entry.stall_progress_eps_m;
if stall_like && ~progressing
    public_vars.path_entry.stall_counter = public_vars.path_entry.stall_counter + 1;
else
    public_vars.path_entry.stall_counter = max(0, public_vars.path_entry.stall_counter - 1);
end

if isfield(public_vars, 'localization_quality')
    public_vars.localization_quality.path_distance_m = path_dist;
end

% Path entry should be conservative about replanning. Once a valid path is
% available, keep trying to merge unless we are clearly not converging.
late_merge = public_vars.nav_state_step >= public_vars.path_entry.max_steps ...
    && path_dist <= 0.30;
if late_merge
    public_vars.path_entry.stall_counter = 0;
    entry_ready = true;
    return;
end

clear_fail = public_vars.nav_state_step >= public_vars.path_entry.max_steps ...
    && path_dist >= public_vars.path_entry.fail_path_dist_m ...
    && heading_err >= public_vars.path_entry.fail_heading_rad ...
    && progress_m <= public_vars.path_entry.fail_progress_m;
stuck_fail = public_vars.path_entry.stall_counter >= public_vars.path_entry.stuck_steps ...
    && path_dist >= public_vars.path_entry.fail_path_dist_m ...
    && heading_err >= public_vars.path_entry.fail_heading_rad ...
    && progress_m <= public_vars.path_entry.fail_progress_m;
if clear_fail || stuck_fail
    entry_failed = true;
end
end

function [seg_idx, path_dist, heading_err] = path_entry_metrics(path, pose)
seg_idx = 1;
path_dist = inf;
heading_err = inf;
if isempty(path) || size(path, 1) < 2 || numel(pose) < 3 || any(~isfinite(pose(1:3)))
    return;
end

valid_rows = all(isfinite(path(:, 1:2)), 2);
if nnz(valid_rows) < 2
    return;
end
if ~all(valid_rows)
    path = path(valid_rows, 1:2);
else
    path = path(:, 1:2);
end

[nearest_idx, path_dist] = nearest_path_index(path, pose(1:2));
if ~isfinite(nearest_idx)
    nearest_idx = 1;
end
nearest_idx = round(nearest_idx);
seg_idx = min(max(nearest_idx, 1), max(1, size(path, 1) - 1));
best_heading_err = inf;
best_seg = seg_idx;
for k = max(1, seg_idx - 1):max(1, min(size(path, 1) - 1, seg_idx + 1))
    p0 = path(k, 1:2);
    p1 = path(k + 1, 1:2);
    if any(~isfinite([p0, p1]))
        continue;
    end
    seg_heading = atan2(p1(2) - p0(2), p1(1) - p0(1));
    seg_err = abs(wrap_to_pi_ws(seg_heading - pose(3)));
    if seg_err < best_heading_err
        best_heading_err = seg_err;
        best_seg = k;
    end
end
seg_idx = best_seg;
heading_err = best_heading_err;
end

function [public_vars, hard_reset, reset_reason] = update_localization_recovery_monitor(read_only_vars, public_vars, lidar_valid)
hard_reset = false;
reset_reason = "stay";

if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end

q = public_vars.localization_quality;
public_vars = update_localization_trace(public_vars);
amb_active = isfield(public_vars, 'environment_ambiguity') ...
    && isfield(public_vars.environment_ambiguity, 'active') ...
    && public_vars.environment_ambiguity.active;
scan_change = inf;
if isfield(public_vars, 'environment_ambiguity') ...
        && isfield(public_vars.environment_ambiguity, 'scan_change') ...
        && isfinite(public_vars.environment_ambiguity.scan_change)
    scan_change = public_vars.environment_ambiguity.scan_change;
end

clear_steps = get_ambiguity_clear_counter_ws(public_vars);
strong_pf = isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= public_vars.localize.contradiction_pf_cluster ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= public_vars.localize.contradiction_pf_mass;
weak_uniqueness = (~isfinite(q.pf_top_ratio) || q.pf_top_ratio < public_vars.localize.contradiction_pf_ratio);
scan_bad = lidar_valid && isfinite(q.scan_match_cost) && q.scan_match_cost >= public_vars.localize.contradiction_scan_cost;
disagree_bad = isfinite(q.disagreement_xy_m) && q.disagreement_xy_m >= public_vars.localize.contradiction_disagree_m;
amb_stuck = amb_active && clear_steps <= public_vars.localize.no_progress_clear_counter_max ...
    && isfinite(scan_change) && scan_change <= public_vars.localize.contradiction_scan_change;
wrong_but_confident = strong_pf && (scan_bad || disagree_bad || (amb_stuck && weak_uniqueness));

if wrong_but_confident
    public_vars.localize.contradiction_counter = public_vars.localize.contradiction_counter + 1;
else
    public_vars.localize.contradiction_counter = max(0, public_vars.localize.contradiction_counter - 1);
end

state_name = string(public_vars.nav_state);
cmd_dist = getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0);
relocalize_no_progress = any(state_name == ["relocalize", "disambiguate"]) ...
    && public_vars.nav_state_step >= public_vars.localize.no_progress_min_steps ...
    && cmd_dist >= public_vars.localize.no_progress_cmd_distance ...
    && clear_steps <= public_vars.localize.no_progress_clear_counter_max ...
    && isfinite(scan_change) && scan_change <= public_vars.localize.no_progress_scan_change;
if relocalize_no_progress
    public_vars.localize.no_progress_counter = public_vars.localize.no_progress_counter + 1;
else
    public_vars.localize.no_progress_counter = max(0, public_vars.localize.no_progress_counter - 1);
end

est_jump_m = getfield_with_default(public_vars.localize, 'trace_last_jump_m', 0);
large_jump_recent = any(state_name == ["localize", "relocalize", "disambiguate"]) ...
    && est_jump_m >= public_vars.localize.large_jump_block_m;
if large_jump_recent
    public_vars.localize.large_jump_hold_counter = max(getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0), ...
        getfield_with_default(public_vars.localize, 'large_jump_hold_steps', 0));
end
side_swap_suspect = any(state_name == ["localize", "relocalize", "disambiguate"]) ...
    && strong_pf ...
    && est_jump_m >= public_vars.localize.sideswap_jump_m ...
    && isfinite(scan_change) && scan_change <= public_vars.localize.sideswap_scan_change_max ...
    && clear_steps <= 1;
side_swap_immediate = any(state_name == ["localize", "relocalize", "disambiguate"]) ...
    && public_vars.nav_state_step <= public_vars.localize.sideswap_immediate_max_steps ...
    && est_jump_m >= public_vars.localize.sideswap_immediate_jump_m ...
    && isfinite(scan_change) && scan_change <= public_vars.localize.sideswap_scan_change_max ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= public_vars.localize.sideswap_immediate_pf_cluster_max ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.45;
large_jump_immediate = state_name == "localize" ...
    && public_vars.nav_state_step <= public_vars.localize.large_jump_immediate_steps ...
    && est_jump_m >= public_vars.localize.large_jump_block_m;
if side_swap_suspect
    public_vars.localize.sideswap_counter = public_vars.localize.sideswap_counter + 1;
else
    public_vars.localize.sideswap_counter = max(0, public_vars.localize.sideswap_counter - 1);
end
if public_vars.localize.sideswap_counter >= public_vars.localize.sideswap_steps
    public_vars.localize.sideswap_hold_counter = public_vars.localize.sideswap_hold_steps;
else
    public_vars.localize.sideswap_hold_counter = max(0, public_vars.localize.sideswap_hold_counter - 1);
end

trace_bbox_diag = getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', inf);
trace_looping = any(state_name == ["relocalize", "disambiguate"]) ...
    && public_vars.nav_state_step >= public_vars.localize.no_progress_min_steps ...
    && cmd_dist >= public_vars.localize.trace_loop_cmd_distance ...
    && isfinite(trace_bbox_diag) && trace_bbox_diag <= public_vars.localize.trace_loop_bbox_diag_m ...
    && clear_steps <= (public_vars.localize.no_progress_clear_counter_max + 1) ...
    && isfinite(scan_change) && scan_change <= public_vars.localize.trace_loop_scan_change;
if trace_looping
    public_vars.localize.trace_loop_counter = public_vars.localize.trace_loop_counter + 1;
else
    public_vars.localize.trace_loop_counter = max(0, public_vars.localize.trace_loop_counter - 1);
end

if public_vars.localize.contradiction_counter >= public_vars.localize.revoke_steps
    public_vars = revoke_localization_commit(public_vars);
end

if side_swap_immediate || large_jump_immediate
    public_vars = revoke_localization_commit(public_vars);
    public_vars.localize.contradiction_counter = 0;
    public_vars.localize.no_progress_counter = 0;
    public_vars.localize.hard_reset_counter = 0;
    public_vars.localize.trace_loop_counter = 0;
    public_vars.localize.sideswap_counter = 0;
    hard_reset = true;
    if state_name == "relocalize"
        reset_reason = "relocalize_hard_reset_bad_hypothesis";
    elseif state_name == "disambiguate"
        reset_reason = "disambiguate_hard_reset_bad_hypothesis";
    else
        reset_reason = "localize_hard_reset_bad_hypothesis";
    end
    return;
end

reset_protected = isfinite(q.scan_match_cost) ...
    && q.scan_match_cost <= min(public_vars.localize.scan_cost_ok, 0.08) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= max(0.28, public_vars.localize.pf_cluster_force_plan + 0.04) ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= max(0.88, public_vars.localize.pf_dominant_mass_good) ...
    && ~weak_uniqueness ...
    && ~large_jump_recent ...
    && ~side_swap_suspect;
hard_reset_signal = public_vars.localize.contradiction_counter >= public_vars.localize.contradiction_steps ...
    || public_vars.localize.sideswap_counter >= public_vars.localize.sideswap_steps ...
    || ((public_vars.localize.no_progress_counter >= public_vars.localize.no_progress_steps ...
        || public_vars.localize.trace_loop_counter >= public_vars.localize.trace_loop_steps) ...
        && ~reset_protected);

if hard_reset_signal
    public_vars.localize.hard_reset_counter = public_vars.localize.hard_reset_counter + 1;
else
    public_vars.localize.hard_reset_counter = max(0, public_vars.localize.hard_reset_counter - 1);
end

if public_vars.localize.hard_reset_counter >= public_vars.localize.hard_reset_steps
    public_vars = revoke_localization_commit(public_vars);
    public_vars.localize.contradiction_counter = 0;
    public_vars.localize.no_progress_counter = 0;
    public_vars.localize.hard_reset_counter = 0;
    public_vars.localize.trace_loop_counter = 0;
    public_vars.localize.sideswap_counter = 0;
    hard_reset = true;
    if state_name == "relocalize"
        reset_reason = "relocalize_hard_reset_bad_hypothesis";
    elseif state_name == "disambiguate"
        reset_reason = "disambiguate_hard_reset_bad_hypothesis";
    else
        reset_reason = "localize_hard_reset_bad_hypothesis";
    end
end
end

function public_vars = revoke_localization_commit(public_vars)
public_vars.localize.stable_counter = 0;
public_vars.localize.fast_recovery = false;
public_vars.localize.allow_global_reseed = true;
public_vars.localize.pose_trace = zeros(0, 2);
public_vars.localize.trace_bbox_diag_m = inf;
public_vars.localize.trace_loop_counter = 0;
public_vars.localize.trace_last_jump_m = 0;
public_vars.localize.sideswap_counter = 0;
public_vars.localize.sideswap_hold_counter = max(getfield_with_default(public_vars.localize, 'sideswap_hold_counter', 0), ...
    getfield_with_default(public_vars.localize, 'sideswap_hold_steps', 0));
public_vars.localize.large_jump_hold_counter = max(getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0), ...
    getfield_with_default(public_vars.localize, 'large_jump_hold_steps', 0));
if isfield(public_vars.localize, 'cmd_turn_accum')
    public_vars.localize.cmd_turn_accum = 0.65 * public_vars.localize.cmd_turn_accum;
end
if isfield(public_vars.localize, 'cmd_distance_accum')
    public_vars.localize.cmd_distance_accum = 0.65 * public_vars.localize.cmd_distance_accum;
end
if isfield(public_vars, 'startup_motion')
    public_vars.startup_motion.commit_heading_rad = nan;
    public_vars.startup_motion.commit_score = -inf;
    public_vars.startup_motion.commit_counter = 0;
    public_vars.startup_motion.filtered_heading_rad = nan;
end
if isfield(public_vars, 'environment_ambiguity') && isfield(public_vars.environment_ambiguity, 'clear_counter')
    public_vars.environment_ambiguity.clear_counter = 0;
end
if isfield(public_vars, 'ambiguity') && isfield(public_vars.ambiguity, 'clear_counter')
    public_vars.ambiguity.clear_counter = 0;
end
if isfield(public_vars, 'localization_commit')
    public_vars.localization_commit.state = "search";
    public_vars.localization_commit.stable_steps = 0;
    public_vars.localization_commit.unstable_steps = 0;
    public_vars.localization_commit.commit_pose = [nan, nan, nan];
    public_vars.localization_commit.commit_score = -inf;
    public_vars.localization_commit.enter_step = getfield_with_default(public_vars, 'global_step', 1);
end
end

function public_vars = update_localization_trace(public_vars)
state_name = string(public_vars.nav_state);
trace_states = ["localize", "relocalize", "disambiguate"];
if ~any(state_name == trace_states) ...
        || ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:2)))
    public_vars.localize.pose_trace = zeros(0, 2);
    public_vars.localize.trace_bbox_diag_m = inf;
    public_vars.localize.trace_last_state = state_name;
    return;
end

last_state = string(getfield_with_default(public_vars.localize, 'trace_last_state', state_name));
trace = getfield_with_default(public_vars.localize, 'pose_trace', zeros(0, 2));
if state_name ~= last_state
    trace = zeros(0, 2);
end
public_vars.localize.trace_last_state = state_name;

xy = public_vars.estimated_pose(1:2);
last_jump = 0;
if isempty(trace) || norm(trace(end, :) - xy) > 0.03
    if ~isempty(trace)
        last_jump = norm(trace(end, :) - xy);
    end
    trace(end + 1, :) = xy; %#ok<AGROW>
end

max_pts = max(10, round(getfield_with_default(public_vars.localize, 'trace_max_points', 60)));
if size(trace, 1) > max_pts
    trace = trace((end - max_pts + 1):end, :);
end
public_vars.localize.pose_trace = trace;
public_vars.localize.trace_last_jump_m = last_jump;

if size(trace, 1) >= 2
    span = max(trace, [], 1) - min(trace, [], 1);
    public_vars.localize.trace_bbox_diag_m = norm(span);
else
    public_vars.localize.trace_bbox_diag_m = inf;
end
end

function [public_vars, tf] = should_finish_localize(public_vars, gnss_valid, lidar_valid)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
confirm_separated = false;
commit_separated = false;
steps = public_vars.nav_state_step;
state_name = string(public_vars.nav_state);
initial_globalize = any(state_name == ["localize", "globalize"]);
informative_motion = startup_localization_informative(public_vars, lidar_valid);
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
side_swap_block = getfield_with_default(public_vars.localize, 'sideswap_hold_counter', 0) > 0;
post_reseed_hold = getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0) > 0;
large_jump_block = getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0) > 0;
extended_commit_guard = getfield_with_default(public_vars.localize, 'extended_commit_guard', false);
trace_bbox_diag = getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', inf);
extended_commit_ready = ~extended_commit_guard ...
    || (isfinite(trace_bbox_diag) ...
        && trace_bbox_diag >= getfield_with_default(public_vars.localize, 'extended_commit_trace_bbox_m', 1.45) ...
        && getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0) >= getfield_with_default(public_vars.localize, 'extended_commit_cmd_distance_m', 1.20) ...
        && getfield_with_default(public_vars.localize, 'cmd_turn_accum', 0) >= getfield_with_default(public_vars.localize, 'extended_commit_turn_rad', 2.40) ...
        && clear_steps >= max(getfield_with_default(public_vars.localize, 'extended_commit_clear_steps', 6), public_vars.ambiguity.clear_required_steps));
commit_failure_count = getfield_with_default(public_vars.localize, 'commit_failure_count', 0);
commit_failure_penalty = min(max(commit_failure_count, 0), getfield_with_default(public_vars.localize, 'commit_failure_max_penalty', 4));
required_finish_clear_steps = public_vars.localize.finish_clear_steps_min + commit_failure_penalty * getfield_with_default(public_vars.localize, 'commit_failure_extra_clear_steps', 3);
commit_motion_ready = true;
failed_commit_pose = getfield_with_default(public_vars.localize, 'last_failed_commit_pose', [nan, nan, nan]);
failed_commit_region_block = false;
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2))) ...
        && numel(failed_commit_pose) >= 2 && all(isfinite(failed_commit_pose(1:2))) ...
        && commit_failure_penalty > 0
    failed_commit_region_block = norm(public_vars.estimated_pose(1:2) - failed_commit_pose(1:2)) ...
        <= getfield_with_default(public_vars.localize, 'failed_commit_block_radius_m', 1.75);
end
commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
hypothesis_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
confirm_separated = hypothesis_separation_ready(q, public_vars.localization_commit, "confirm");
commit_separated = hypothesis_separation_ready(q, public_vars.localization_commit, "commit");
single_hypothesis = isfinite(hypothesis_count) && hypothesis_count <= 1 ...
    && isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= public_vars.localize.pf_top_ratio_single_ok;
few_hypotheses = isfinite(hypothesis_count) && hypothesis_count <= 2;
pf_unique_ok = (isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= public_vars.localize.pf_top_ratio_finish_ok) ...
    || single_hypothesis;
pf_unique_force = (isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= public_vars.localize.pf_top_ratio_finish_force) ...
    || single_hypothesis;
clear_ready = clear_steps >= required_finish_clear_steps;
symmetry_hold_block = large_jump_block ...
    || (post_reseed_hold && (ambiguity_active_or_ws(public_vars) || ~single_hypothesis || ~clear_ready));
trend_ok = (~isfield(q, 'scan_match_trend') || ~isfinite(q.scan_match_trend) || q.scan_match_trend <= 0.02) ...
    && (~isfield(q, 'pf_cluster_trend') || ~isfinite(q.pf_cluster_trend) || q.pf_cluster_trend <= 0.02) ...
    && (~isfield(q, 'disagreement_trend') || ~isfinite(q.disagreement_trend) || q.disagreement_trend <= 0.05);
good_now = false;
fusion_consistent = ~isfield(q, 'lidar_valid') || ~q.lidar_valid ...
    || ~isfinite(q.disagreement_xy_m) ...
    || q.disagreement_xy_m <= public_vars.localize.disagree_ok;
committed_ready = commit_state == "committed";

if ~initial_globalize ...
        && commit_state == "committed" ...
        && steps >= 6 ...
        && isfinite(q.scan_match_cost) && q.scan_match_cost <= public_vars.localize.scan_cost_ok ...
        && clear_steps >= required_finish_clear_steps ...
        && trend_ok ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && ~failed_commit_region_block ...
        && commit_separated
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

if gnss_valid && isfinite(q.kf_var_xy_mean) ...
        && q.kf_var_xy_mean <= public_vars.localize.kf_var_good ...
        && fusion_consistent ...
        && (~lidar_valid || (pf_unique_ok && clear_ready)) ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && confirm_separated
    good_now = true;
end

if lidar_valid && isfinite(q.pf_cluster_radius_m)
    if q.pf_cluster_radius_m <= public_vars.localize.pf_cluster_good ...
            && q.pf_dominant_mass >= public_vars.localize.pf_dominant_mass_good ...
            && pf_unique_ok ...
            && isfinite(q.scan_match_cost) ...
            && q.scan_match_cost <= public_vars.localize.scan_cost_good ...
            && (~isfield(public_vars, 'environment_ambiguity') || ~public_vars.environment_ambiguity.active) ...
            && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_good) ...
            && clear_ready ...
            && trend_ok ...
            && ~side_swap_block ...
            && ~symmetry_hold_block ...
            && confirm_separated
        good_now = true;
    end
end

if good_now
    public_vars.localize.stable_counter = public_vars.localize.stable_counter + 1;
else
    public_vars.localize.stable_counter = 0;
end

if gnss_valid && isfinite(q.kf_var_xy_mean) ...
        && q.kf_var_xy_mean <= public_vars.localize.kf_var_ok ...
        && fusion_consistent ...
        && (~lidar_valid || (pf_unique_ok && clear_ready)) ...
        && steps >= 12 ...
        && ~side_swap_block ...
        && ~symmetry_hold_block
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

if public_vars.localize.fast_recovery ...
        && lidar_valid ...
        && steps >= public_vars.localize.fast_exit_min_steps ...
        && public_vars.localize.stable_counter >= public_vars.localize.fast_exit_stable_steps ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= public_vars.localize.fast_exit_pf_cluster ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= public_vars.localize.fast_exit_pf_mass ...
        && is_localization_confident_for_plan(public_vars, true) ...
        && informative_motion ...
        && public_vars.localize.cmd_distance_accum >= public_vars.localize.fast_exit_cmd_distance ...
        && public_vars.localize.cmd_turn_accum >= public_vars.localize.fast_exit_cmd_turn ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && commit_separated
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

late_pf_ready = ~initial_globalize ...
        && lidar_valid ...
        && committed_ready ...
        && extended_commit_ready ...
        && steps >= public_vars.localize.late_exit_min_steps ...
        && public_vars.localize.stable_counter >= public_vars.localize.late_exit_stable_steps ...
        && get_ambiguity_clear_counter_ws(public_vars) >= public_vars.ambiguity.clear_required_steps ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= public_vars.localize.late_exit_pf_cluster ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= public_vars.localize.late_exit_pf_mass ...
        && pf_unique_force ...
        && isfinite(q.scan_match_cost) && q.scan_match_cost <= min(public_vars.localize.late_exit_scan_cost, public_vars.localize.recovery_exit_scan_cost) ...
        && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_ok) ...
        && (~isfield(public_vars, 'environment_ambiguity') || ~public_vars.environment_ambiguity.active) ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && commit_separated;
if late_pf_ready
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

recovery_pf_ready = ~initial_globalize ...
    && lidar_valid ...
    && committed_ready ...
    && extended_commit_ready ...
    && steps >= max(90, ceil(0.40 * public_vars.localize.min_steps)) ...
    && public_vars.localize.stable_counter >= max(8, ceil(0.45 * public_vars.localize.stable_steps_required)) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.28 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.82 ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= public_vars.localize.recovery_exit_scan_cost ...
    && (few_hypotheses || pf_unique_ok) ...
    && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_ok) ...
    && trend_ok ...
    && informative_motion ...
    && public_vars.localize.cmd_distance_accum >= 0.90 * public_vars.localize.min_cmd_distance ...
    && public_vars.localize.cmd_turn_accum >= 0.50 * public_vars.localize.min_cmd_turn ...
    && (~isfield(public_vars, 'environment_ambiguity') ...
        || ~public_vars.environment_ambiguity.active ...
        || clear_steps >= 1) ...
    && ~side_swap_block ...
    && ~symmetry_hold_block ...
    && commit_separated;
if recovery_pf_ready
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

long_run_snap_ready = lidar_valid ...
    && committed_ready ...
    && extended_commit_ready ...
    && steps >= max(160, ceil(0.75 * public_vars.localize.min_steps)) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.26 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.88 ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= 0.01 ...
    && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_good) ...
    && trend_ok ...
    && public_vars.localize.cmd_distance_accum >= 0.75 ...
    && public_vars.localize.cmd_turn_accum >= 0.45 ...
    && ~side_swap_block ...
    && ~symmetry_hold_block;
if long_run_snap_ready
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

strong_pf_ready = ~initial_globalize ...
    && lidar_valid ...
    && committed_ready ...
    && extended_commit_ready ...
    && steps >= max(70, ceil(0.35 * public_vars.localize.min_steps)) ...
    && public_vars.localize.stable_counter >= max(6, ceil(0.4 * public_vars.localize.stable_steps_required)) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.24 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90 ...
    && pf_unique_force ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= public_vars.localize.recovery_exit_scan_cost ...
    && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_ok) ...
    && trend_ok ...
    && informative_motion ...
    && clear_ready ...
    && public_vars.localize.cmd_distance_accum >= 0.75 * public_vars.localize.min_cmd_distance ...
    && public_vars.localize.cmd_turn_accum >= 0.55 * public_vars.localize.min_cmd_turn ...
    && ~side_swap_block ...
    && ~symmetry_hold_block ...
    && commit_separated;
if strong_pf_ready
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

if steps >= public_vars.localize.min_steps ...
        && committed_ready ...
        && extended_commit_ready ...
        && public_vars.localize.stable_counter >= public_vars.localize.stable_steps_required ...
        && is_localization_confident_for_plan(public_vars, true) ...
        && informative_motion ...
        && get_ambiguity_clear_counter_ws(public_vars) >= public_vars.ambiguity.clear_required_steps ...
        && public_vars.localize.cmd_distance_accum >= public_vars.localize.min_cmd_distance ...
        && public_vars.localize.cmd_turn_accum >= public_vars.localize.min_cmd_turn ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && commit_separated
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

if lidar_valid && steps >= public_vars.localize.max_steps ...
        && committed_ready ...
        && extended_commit_ready ...
        && is_localization_confident_for_plan(public_vars, true) ...
        && informative_motion ...
        && public_vars.localize.cmd_distance_accum >= public_vars.localize.force_plan_cmd_distance ...
        && public_vars.localize.stable_counter >= public_vars.localize.force_plan_stable_steps ...
        && ~side_swap_block ...
        && ~symmetry_hold_block ...
        && commit_separated
    tf = true;
    public_vars.localize.fast_recovery = false;
    return;
end

if steps >= public_vars.localize.max_steps
    if gnss_valid && isfinite(q.kf_var_xy_mean) ...
            && q.kf_var_xy_mean <= public_vars.localize.kf_var_ok ...
            && fusion_consistent ...
            && extended_commit_ready ...
            && (~lidar_valid || (pf_unique_ok && clear_ready)) ...
            && confirm_separated
        tf = true;
        public_vars.localize.fast_recovery = false;
        return;
    end
    if lidar_valid && isfinite(q.pf_cluster_radius_m) ...
            && committed_ready ...
            && extended_commit_ready ...
            && q.pf_cluster_radius_m <= public_vars.localize.pf_cluster_ok ...
            && q.pf_dominant_mass >= public_vars.localize.pf_dominant_mass_ok ...
            && pf_unique_ok ...
            && isfinite(q.scan_match_cost) ...
            && q.scan_match_cost <= public_vars.localize.scan_cost_ok ...
            && (~isfield(public_vars, 'environment_ambiguity') || ~public_vars.environment_ambiguity.active) ...
            && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_ok) ...
            && informative_motion ...
        && get_ambiguity_clear_counter_ws(public_vars) >= public_vars.ambiguity.clear_required_steps ...
            && public_vars.localize.cmd_distance_accum >= public_vars.localize.min_cmd_distance ...
            && public_vars.localize.cmd_turn_accum >= public_vars.localize.min_cmd_turn ...
            && ~side_swap_block ...
            && commit_separated
        tf = true;
        public_vars.localize.fast_recovery = false;
        return;
    end
end

if steps >= public_vars.localize.hard_max_steps
    tf = lidar_valid ...
        && committed_ready ...
        && extended_commit_ready ...
        && public_vars.localize.stable_counter >= public_vars.localize.force_plan_stable_steps ...
        && is_localization_confident_for_plan(public_vars, true) ...
        && informative_motion ...
        && ~side_swap_block ...
        && commit_separated;
    if tf
        public_vars.localize.fast_recovery = false;
    end
end
end

function public_vars = update_localization_commit_state(read_only_vars, public_vars, gnss_valid, lidar_valid)
if ~isfield(public_vars, 'localization_commit') || isempty(public_vars.localization_commit)
    return;
end
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end

q = public_vars.localization_quality;
lc = public_vars.localization_commit;
confirm_separated = false;
commit_separated = false;
h1 = get_primary_hypothesis_metrics(q);
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
cmd_distance_accum = getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0);
cmd_turn_accum = getfield_with_default(public_vars.localize, 'cmd_turn_accum', 0);
ambiguity_active = isfield(public_vars, 'environment_ambiguity') ...
    && isfield(public_vars.environment_ambiguity, 'active') ...
    && public_vars.environment_ambiguity.active;
commit_failure_count = getfield_with_default(public_vars.localize, 'commit_failure_count', 0);
commit_failure_penalty = min(max(commit_failure_count, 0), getfield_with_default(public_vars.localize, 'commit_failure_max_penalty', 4));
required_confirm_clear_steps = 1 + commit_failure_penalty * getfield_with_default(public_vars.localize, 'commit_failure_extra_clear_steps', 3);
required_commit_clear_steps = max(2, public_vars.localize.finish_clear_steps_min) + commit_failure_penalty * getfield_with_default(public_vars.localize, 'commit_failure_extra_clear_steps', 3);
commit_motion_ready = true;
failed_commit_pose = getfield_with_default(public_vars.localize, 'last_failed_commit_pose', [nan, nan, nan]);
failed_commit_region_block = false;
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2))) ...
        && numel(failed_commit_pose) >= 2 && all(isfinite(failed_commit_pose(1:2))) ...
        && commit_failure_penalty > 0
    failed_commit_region_block = norm(public_vars.estimated_pose(1:2) - failed_commit_pose(1:2)) ...
        <= getfield_with_default(public_vars.localize, 'failed_commit_block_radius_m', 1.75);
end
post_reseed_hold = getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0) > 0;
side_swap_block = getfield_with_default(public_vars.localize, 'sideswap_hold_counter', 0) > 0;
large_jump_block = getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0) > 0;
est_jump_m = getfield_with_default(public_vars.localize, 'trace_last_jump_m', 0);
extended_commit_guard = getfield_with_default(public_vars.localize, 'extended_commit_guard', false);
trace_bbox_diag = getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', inf);
extended_commit_ready = ~extended_commit_guard ...
    || (isfinite(trace_bbox_diag) ...
        && trace_bbox_diag >= getfield_with_default(public_vars.localize, 'extended_commit_trace_bbox_m', 1.45) ...
        && cmd_distance_accum >= getfield_with_default(public_vars.localize, 'extended_commit_cmd_distance_m', 1.20) ...
        && cmd_turn_accum >= getfield_with_default(public_vars.localize, 'extended_commit_turn_rad', 2.40) ...
        && clear_steps >= max(getfield_with_default(public_vars.localize, 'extended_commit_clear_steps', 6), public_vars.ambiguity.clear_required_steps));
commit_motion_ready = cmd_distance_accum >= max(2.05, 1.15 * getfield_with_default(public_vars.localize, 'min_informative_cmd_distance', 1.0)) ...
    && cmd_turn_accum >= max(2.20, 0.85 * getfield_with_default(public_vars.localize, 'min_informative_cmd_turn', 0.45 * pi));
if commit_failure_penalty > 0
    commit_motion_ready = commit_motion_ready ...
        && isfinite(trace_bbox_diag) ...
        && trace_bbox_diag >= max(1.0, getfield_with_default(public_vars.localize, 'extended_commit_trace_bbox_m', 1.45) - 0.20);
end
late_recovery_ready = false;
scan_ok = isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.scan_cost_confirm;
scan_commit = isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.scan_cost_commit;
hypothesis_age_confirm_ok = (h1.valid && h1.age >= lc.hypothesis_age_confirm) ...
    || (~h1.valid && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.78);
hypothesis_age_commit_ok = (h1.valid && h1.age >= lc.hypothesis_age_commit) ...
    || (~h1.valid && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.88);
hypothesis_stability_confirm_ok = (h1.valid && h1.stability >= lc.hypothesis_stability_confirm) ...
    || (~h1.valid && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.74 ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= max(0.40, lc.pf_cluster_confirm_m + 0.10));
hypothesis_stability_commit_ok = (h1.valid && h1.stability >= lc.hypothesis_stability_commit) ...
    || (~h1.valid && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.86 ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= max(0.30, lc.pf_cluster_commit_m + 0.12));
hypothesis_scan_confirm_ok = (h1.valid && h1.scan_score >= 0.40) ...
    || (~h1.valid && isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.scan_cost_confirm);
hypothesis_scan_commit_ok = (h1.valid && h1.scan_score >= 0.55) ...
    || (~h1.valid && isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.scan_cost_commit);
hypothesis_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
startup_commit_state = string(getfield_with_default(public_vars, 'nav_state', "localize")) == "localize";
commit_distance_threshold = getfield_with_default(public_vars.localize, 'recovery_commit_cmd_distance_m', 2.00);
commit_trace_threshold = getfield_with_default(public_vars.localize, 'recovery_commit_trace_bbox_m', 1.00);
if getfield_with_default(public_vars.localize, 'fast_recovery', false)
    commit_distance_threshold = max(commit_distance_threshold, ...
        getfield_with_default(public_vars.localize, 'fast_recovery_commit_cmd_distance_m', 4.00));
    commit_trace_threshold = max(commit_trace_threshold, ...
        getfield_with_default(public_vars.localize, 'fast_recovery_commit_trace_bbox_m', 2.20));
end
recovery_commit_motion_ready = startup_commit_state ...
    || (cmd_distance_accum >= commit_distance_threshold ...
        && isfinite(trace_bbox_diag) ...
        && trace_bbox_diag >= commit_trace_threshold);
single_hypothesis_raw = isfinite(hypothesis_count) && hypothesis_count <= lc.pf_hyp_commit_max ...
    && (h1.valid && h1.stability >= lc.hypothesis_stability_confirm || isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90);
single_hypothesis = single_hypothesis_raw ...
    && (hypothesis_age_commit_ok || isfinite(q.pf_top_ratio) && q.pf_top_ratio >= lc.pf_top_ratio_single_commit);
strong_single_hypothesis = single_hypothesis ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= max(0.28, lc.pf_cluster_commit_m + 0.04) ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90 ...
    && scan_ok;
confirm_separated = hypothesis_separation_ready(q, lc, "confirm");
commit_separated = hypothesis_separation_ready(q, lc, "commit");
late_recovery_ready = string(getfield_with_default(public_vars, 'nav_state', "localize")) == "relocalize" ...
    && clear_steps >= getfield_with_default(public_vars.localize, 'late_recovery_clear_steps', 18) ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= getfield_with_default(public_vars.localize, 'late_recovery_scan_cost', 0.08) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= getfield_with_default(public_vars.localize, 'late_recovery_pf_cluster_m', 0.22) ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= getfield_with_default(public_vars.localize, 'late_recovery_pf_mass', 0.96) ...
    && ((isfinite(hypothesis_count) && hypothesis_count <= 1) ...
        || (isfinite(q.pf_top_ratio) && q.pf_top_ratio >= getfield_with_default(public_vars.localize, 'late_recovery_pf_top_ratio', 1.12))) ...
    && (~getfield_with_default(q, 'gnss_valid', false) ...
        || (isfinite(q.disagreement_xy_m) && q.disagreement_xy_m <= getfield_with_default(public_vars.localize, 'late_recovery_disagree_m', 0.55))) ...
    && est_jump_m <= getfield_with_default(public_vars.localize, 'late_recovery_est_jump_m', 0.20) ...
    && getfield_with_default(public_vars.localize, 'contradiction_counter', 0) <= getfield_with_default(public_vars.localize, 'late_recovery_max_contradictions', 1) ...
    && getfield_with_default(public_vars.localize, 'no_progress_counter', 0) <= getfield_with_default(public_vars.localize, 'late_recovery_max_no_progress', 2) ...
    && ~failed_commit_region_block ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && commit_separated;
ambiguity_block = ambiguity_active && ~strong_single_hypothesis;
single_hypothesis_commit_ready = lidar_valid ...
    && startup_commit_state ...
    && single_hypothesis_raw ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= lc.pf_cluster_single_commit_m ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= lc.pf_mass_single_commit ...
    && ((isfinite(q.pf_top_ratio) && q.pf_top_ratio >= lc.pf_top_ratio_single_commit) || single_hypothesis) ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.scan_cost_single_commit ...
    && hypothesis_age_confirm_ok ...
    && hypothesis_stability_confirm_ok ...
    && hypothesis_scan_confirm_ok ...
    && clear_steps >= lc.single_commit_clear_steps ...
    && isfinite(trace_bbox_diag) && trace_bbox_diag >= lc.single_commit_min_trace_bbox_m ...
    && ~failed_commit_region_block ...
    && ~ambiguity_block ...
    && extended_commit_ready ...
    && commit_motion_ready ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.25 ...
    && commit_separated;
exploratory_commit_ready = false;
confirm_unique = (isfinite(q.pf_top_ratio) && q.pf_top_ratio >= lc.pf_top_ratio_confirm) ...
    || (isfinite(hypothesis_count) && hypothesis_count <= lc.pf_hyp_confirm_max ...
        && hypothesis_age_confirm_ok && hypothesis_stability_confirm_ok);
commit_unique = (isfinite(q.pf_top_ratio) && q.pf_top_ratio >= lc.pf_top_ratio_commit) ...
    || (single_hypothesis_raw && hypothesis_age_commit_ok && hypothesis_stability_commit_ok);
strong_instant_confirm = lidar_valid ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.24 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.95 ...
    && isfinite(q.pf_top_ratio) && q.pf_top_ratio >= max(lc.pf_top_ratio_single_confirm, 1.16) ...
    && scan_ok ...
    && clear_steps >= 3 ...
    && ~failed_commit_region_block ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.20 ...
    && confirm_separated;
strong_instant_commit = lidar_valid ...
    && startup_commit_state ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.20 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.97 ...
    && isfinite(q.pf_top_ratio) && q.pf_top_ratio >= max(lc.pf_top_ratio_single_commit, 1.18) ...
    && scan_commit ...
    && clear_steps >= 5 ...
    && ~failed_commit_region_block ...
    && extended_commit_ready ...
    && commit_motion_ready ...
    && ~ambiguity_block ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.15 ...
    && commit_separated;
pf_confirm = lidar_valid ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= lc.pf_cluster_confirm_m ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= lc.pf_mass_confirm ...
    && confirm_unique ...
    && scan_ok ...
    && hypothesis_age_confirm_ok ...
    && hypothesis_stability_confirm_ok ...
    && hypothesis_scan_confirm_ok ...
    && clear_steps >= required_confirm_clear_steps ...
    && ~failed_commit_region_block ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.60 ...
    && confirm_separated;
pf_commit = lidar_valid ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= lc.pf_cluster_commit_m ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= lc.pf_mass_commit ...
    && commit_unique ...
    && scan_commit ...
    && hypothesis_age_commit_ok ...
    && hypothesis_stability_commit_ok ...
    && hypothesis_scan_commit_ok ...
    && ~ambiguity_block ...
    && clear_steps >= required_commit_clear_steps ...
    && ~failed_commit_region_block ...
    && extended_commit_ready ...
    && recovery_commit_motion_ready ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.35 ...
    && commit_separated;
gnss_commit = gnss_valid ...
    && isfinite(q.kf_var_xy_mean) && q.kf_var_xy_mean <= public_vars.localize.kf_var_good ...
    && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_good);
confirm_now = pf_confirm || gnss_commit;
commit_now = pf_commit || single_hypothesis_commit_ready || gnss_commit || late_recovery_ready;
confirm_hold = lidar_valid ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.22 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.94 ...
    && scan_ok ...
    && ~post_reseed_hold ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && est_jump_m <= 0.25;
score = localization_commit_score(q, clear_steps, ambiguity_block);

switch string(lc.state)
    case "search"
        if strong_instant_commit
            lc.state = "committed";
            lc.stable_steps = lc.confirm_required_steps;
            lc.unstable_steps = 0;
            lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
            if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose)
                lc.commit_pose = public_vars.estimated_pose;
            end
            lc.commit_score = score;
        elseif confirm_now || strong_instant_confirm
            lc.state = "confirm";
            lc.stable_steps = 1 + double(strong_instant_confirm);
            lc.unstable_steps = 0;
            lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
        end
    case "confirm"
        if post_reseed_hold || side_swap_block || large_jump_block || est_jump_m > 0.80
            lc.state = "search";
            lc.stable_steps = 0;
            lc.unstable_steps = 0;
            lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
            lc.commit_pose = [nan, nan, nan];
            lc.commit_score = -inf;
        elseif commit_now || strong_instant_commit
            lc.stable_steps = lc.stable_steps + 1;
        elseif confirm_now || strong_instant_confirm
            lc.stable_steps = lc.stable_steps + 1;
            lc.unstable_steps = max(0, lc.unstable_steps - 1);
        elseif confirm_hold
            lc.stable_steps = min(lc.confirm_required_steps, lc.stable_steps + 1);
            lc.unstable_steps = max(0, lc.unstable_steps - 1);
        else
            lc.unstable_steps = lc.unstable_steps + 1;
            if lc.unstable_steps >= 2
                lc.state = "search";
                lc.stable_steps = 0;
                lc.unstable_steps = 0;
                lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
            end
        end
        if lc.stable_steps >= lc.confirm_required_steps && commit_now
            lc.state = "committed";
            lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
            if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose)
                lc.commit_pose = public_vars.estimated_pose;
            end
            lc.commit_score = score;
            lc.unstable_steps = 0;
        end
    otherwise
        if post_reseed_hold || side_swap_block || large_jump_block || est_jump_m > 0.80
            lc.state = "search";
            lc.stable_steps = 0;
            lc.unstable_steps = 0;
            lc.commit_pose = [nan, nan, nan];
            lc.commit_score = -inf;
            lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
        elseif commit_now
            lc.stable_steps = min(lc.confirm_required_steps + 4, lc.stable_steps + 1);
            lc.unstable_steps = 0;
            if isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose) ...
                    && score >= lc.commit_score - 0.08
                lc.commit_pose = public_vars.estimated_pose;
                lc.commit_score = max(lc.commit_score, score);
            end
        else
            lc.unstable_steps = lc.unstable_steps + 1;
            if lc.unstable_steps >= lc.drop_required_steps
                lc.state = "search";
                lc.stable_steps = 0;
                lc.unstable_steps = 0;
                lc.commit_pose = [nan, nan, nan];
                lc.commit_score = -inf;
                lc.enter_step = getfield_with_default(public_vars, 'global_step', 1);
            end
        end
end

public_vars.localization_commit = lc;
end

function s = localization_commit_score(q, clear_steps, ambiguity_active)
s = 0;
h1 = get_primary_hypothesis_metrics(q);
if isfinite(q.pf_dominant_mass)
    s = s + 1.5 * q.pf_dominant_mass;
end
if isfinite(q.pf_top_ratio)
    s = s + 0.4 * min(q.pf_top_ratio, 2.0);
end
if isfinite(q.pf_cluster_radius_m)
    s = s - 2.2 * q.pf_cluster_radius_m;
end
if isfinite(q.scan_match_cost)
    s = s - 1.8 * q.scan_match_cost;
end
if h1.valid
    s = s + 0.80 * h1.mass;
    s = s + 0.55 * h1.scan_score;
    s = s + 0.35 * h1.stability;
    s = s + 0.05 * min(h1.age, 12);
    s = s - 0.60 * h1.radius_m;
end
s = s + 0.08 * clear_steps - 0.35 * double(ambiguity_active);
end

function h1 = get_primary_hypothesis_metrics(q)
h1 = struct('valid', false, 'mass', nan, 'radius_m', nan, 'scan_score', nan, 'age', 0, 'stability', 0);
if ~isstruct(q)
    return;
end
h1.mass = getfield_with_default(q, 'h1_mass', nan);
h1.radius_m = getfield_with_default(q, 'h1_radius_m', nan);
h1.scan_score = getfield_with_default(q, 'h1_scan_score', nan);
h1.age = getfield_with_default(q, 'h1_age', 0);
h1.stability = getfield_with_default(q, 'h1_stability', 0);
h1.valid = isfinite(h1.mass) && isfinite(h1.radius_m) && isfinite(h1.scan_score);
end

function tf = hypothesis_separation_ready(q, lc, mode)
tf = true;
if ~isstruct(q) || ~isstruct(lc)
    return;
end
mass_ratio = getfield_with_default(q, 'h12_mass_ratio', nan);
scan_gap = getfield_with_default(q, 'h12_scan_gap', nan);
stability_gap = getfield_with_default(q, 'h12_stability_gap', nan);
pose_sep = getfield_with_default(q, 'h2_pose_sep_m', nan);

switch string(mode)
    case "commit"
        max_mass_ratio = getfield_with_default(lc, 'h2_mass_ratio_commit_max', 0.68);
        min_scan_gap = getfield_with_default(lc, 'h2_scan_gap_commit_min', 0.10);
        min_stability_gap = getfield_with_default(lc, 'h2_stability_gap_commit_min', 0.08);
        min_pose_sep = getfield_with_default(lc, 'h2_pose_sep_commit_min', 1.20);
    otherwise
        max_mass_ratio = getfield_with_default(lc, 'h2_mass_ratio_confirm_max', 0.82);
        min_scan_gap = getfield_with_default(lc, 'h2_scan_gap_confirm_min', 0.06);
        min_stability_gap = 0;
        min_pose_sep = 0;
end

if ~isfinite(mass_ratio)
    return;
end

tf = mass_ratio <= max_mass_ratio;
if tf && isfinite(scan_gap)
    tf = scan_gap >= min_scan_gap;
end
if tf && min_stability_gap > 0 && isfinite(stability_gap)
    tf = stability_gap >= min_stability_gap;
end
if tf && min_pose_sep > 0 && isfinite(pose_sep)
    tf = pose_sep >= min_pose_sep;
end
end

function value = get_nested_struct_or(s, struct_field, field_name, fallback)
value = fallback;
if isstruct(s) && isfield(s, struct_field) && isstruct(s.(struct_field)) ...
        && isfield(s.(struct_field), field_name) && ~isempty(s.(struct_field).(field_name))
    value = s.(struct_field).(field_name);
end
end

function tf = ambiguity_active_or_ws(public_vars)
tf = isfield(public_vars, 'environment_ambiguity') ...
    && isfield(public_vars.environment_ambiguity, 'active') ...
    && public_vars.environment_ambiguity.active;
end

function ro = apply_active_verify_goal_ws(read_only_vars, public_vars)
ro = read_only_vars;
if ~isfield(public_vars, 'verify') || ~public_vars.verify.active ...
        || ~isfield(public_vars.verify, 'goal_xy') || numel(public_vars.verify.goal_xy) < 2 ...
        || any(~isfinite(public_vars.verify.goal_xy(1:2)))
    return;
end
ro.map.goal = public_vars.verify.goal_xy(1:2);
end

function public_vars = abort_verify_goal_ws(public_vars)
if ~isfield(public_vars, 'verify')
    return;
end
public_vars.verify.active = false;
public_vars.verify.goal_xy = [nan, nan];
public_vars.verify.start_xy = [nan, nan];
public_vars.verify.navigation_allow_counter = 0;
end

function public_vars = prepare_verify_goal_ws(read_only_vars, public_vars)
if ~isfield(public_vars, 'verify')
    return;
end
public_vars.verify.active = false;
public_vars.verify.goal_xy = [nan, nan];
public_vars.verify.start_xy = [nan, nan];
q = getfield_with_default(public_vars, 'localization_quality', struct());
strong_skip_verify = isfield(q, 'pf_hypothesis_count') && isfinite(q.pf_hypothesis_count) ...
    && q.pf_hypothesis_count <= 1 ...
    && isfield(q, 'pf_cluster_radius_m') && isfinite(q.pf_cluster_radius_m) ...
    && q.pf_cluster_radius_m <= getfield_with_default(public_vars.verify, 'skip_pf_cluster_m', 0.18) ...
    && isfield(q, 'pf_dominant_mass') && isfinite(q.pf_dominant_mass) ...
    && q.pf_dominant_mass >= getfield_with_default(public_vars.verify, 'skip_pf_mass', 0.995) ...
    && isfield(q, 'scan_match_cost') && isfinite(q.scan_match_cost) ...
    && q.scan_match_cost <= getfield_with_default(public_vars.verify, 'skip_scan_cost', public_vars.localize.scan_cost_ok) ...
    && ~ambiguity_active_or_ws(public_vars) ...
    && getfield_with_default(public_vars.localize, 'commit_failure_count', 0) <= 0;
if strong_skip_verify
    return;
end
tmp_pub = public_vars;
tmp_pub.verify.selecting = true;
tmp_pub.disambiguate.min_goal_distance_m = getfield_with_default(public_vars.verify, 'min_goal_distance_m', 1.4);
tmp_pub.disambiguate.max_goal_distance_m = getfield_with_default(public_vars.verify, 'max_goal_distance_m', 3.4);
tmp_pub.disambiguate.goal_length_penalty = 1.6 * getfield_with_default(public_vars.disambiguate, 'goal_length_penalty', 0.08);
[goal_xy, found] = find_informative_goal(read_only_vars, tmp_pub);
if ~found || any(~isfinite(goal_xy))
    return;
end
final_goal = read_only_vars.map.goal(1:2);
if norm(goal_xy(:)' - final_goal(:)') < getfield_with_default(public_vars.verify, 'min_goal_separation_from_final_m', 1.0)
    return;
end
public_vars.verify.goal_xy = goal_xy(:)';
public_vars.verify.start_xy = public_vars.estimated_pose(1:2);
public_vars.verify.active = true;
public_vars.verify.navigation_allow_counter = getfield_with_default(public_vars.verify, 'navigation_allow_steps', 180);
end

function tf = is_localization_confident_for_plan(public_vars, ignore_ambiguity)
tf = false;
if nargin < 2
    ignore_ambiguity = false;
end
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
if ~isfield(q, 'pose_valid') || ~q.pose_valid
    return;
end
goal_dist_est = inf;
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2))) ...
        && isfield(public_vars, 'goal') && numel(public_vars.goal) >= 2 && all(isfinite(public_vars.goal(1:2)))
    goal_dist_est = norm(public_vars.estimated_pose(1:2) - public_vars.goal(1:2));
end

ambiguity_ok = true;
if ~ignore_ambiguity
    ambiguity_ok = ~isfield(public_vars, 'environment_ambiguity') || ~public_vars.environment_ambiguity.active;
end
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
commit_failure_count = getfield_with_default(public_vars.localize, 'commit_failure_count', 0);
commit_failure_penalty = min(max(commit_failure_count, 0), getfield_with_default(public_vars.localize, 'commit_failure_max_penalty', 4));
required_finish_clear_steps = public_vars.localize.finish_clear_steps_min + commit_failure_penalty * getfield_with_default(public_vars.localize, 'commit_failure_extra_clear_steps', 3);
commit_motion_ready = true;
failed_commit_pose = getfield_with_default(public_vars.localize, 'last_failed_commit_pose', [nan, nan, nan]);
failed_commit_region_block = false;
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2))) ...
        && numel(failed_commit_pose) >= 2 && all(isfinite(failed_commit_pose(1:2))) ...
        && commit_failure_penalty > 0
    failed_commit_region_block = norm(public_vars.estimated_pose(1:2) - failed_commit_pose(1:2)) ...
        <= getfield_with_default(public_vars.localize, 'failed_commit_block_radius_m', 1.75);
end
clear_ready = clear_steps >= required_finish_clear_steps;
side_swap_block = getfield_with_default(public_vars.localize, 'sideswap_hold_counter', 0) > 0;
post_reseed_hold = getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0) > 0;
large_jump_block = getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0) > 0;
hypothesis_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
confirm_separated = hypothesis_separation_ready(q, public_vars.localization_commit, "confirm");
commit_separated = hypothesis_separation_ready(q, public_vars.localization_commit, "commit");
single_hypothesis = isfinite(hypothesis_count) && hypothesis_count <= 1;
symmetry_hold_block = large_jump_block ...
    || (post_reseed_hold && (ambiguity_active_or_ws(public_vars) || ~single_hypothesis || ~clear_ready));
pf_unique_force = (isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= public_vars.localize.pf_top_ratio_finish_force) ...
    || single_hypothesis;
pf_unique_ok = (isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= public_vars.localize.pf_top_ratio_finish_ok) ...
    || single_hypothesis;
fusion_consistent = ~isfield(q, 'lidar_valid') || ~q.lidar_valid ...
    || ~isfinite(q.disagreement_xy_m) ...
    || q.disagreement_xy_m <= public_vars.localize.disagree_force_plan;
informative_motion = startup_localization_informative(public_vars, isfield(q, 'lidar_valid') && q.lidar_valid);
finish_distance_threshold = getfield_with_default(public_vars.localize, 'recovery_commit_cmd_distance_m', 2.00);
finish_trace_threshold = getfield_with_default(public_vars.localize, 'recovery_commit_trace_bbox_m', 1.00);
if getfield_with_default(public_vars.localize, 'fast_recovery', false)
    finish_distance_threshold = max(finish_distance_threshold, ...
        getfield_with_default(public_vars.localize, 'fast_recovery_commit_cmd_distance_m', 4.00));
    finish_trace_threshold = max(finish_trace_threshold, ...
        getfield_with_default(public_vars.localize, 'fast_recovery_commit_trace_bbox_m', 2.20));
end
recovery_finish_motion_ready = string(getfield_with_default(public_vars, 'nav_state', "localize")) == "localize" ...
    || (getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0) >= finish_distance_threshold ...
        && isfinite(getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', nan)) ...
        && getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', nan) >= finish_trace_threshold);
late_recovery_finish_ready = string(getfield_with_default(public_vars, 'nav_state', "localize")) == "relocalize" ...
    && clear_steps >= getfield_with_default(public_vars.localize, 'late_recovery_clear_steps', 18) ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= getfield_with_default(public_vars.localize, 'late_recovery_scan_cost', 0.08) ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= getfield_with_default(public_vars.localize, 'late_recovery_pf_cluster_m', 0.22) ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= getfield_with_default(public_vars.localize, 'late_recovery_pf_mass', 0.96) ...
    && ((isfinite(hypothesis_count) && hypothesis_count <= 1) ...
        || (isfinite(q.pf_top_ratio) && q.pf_top_ratio >= getfield_with_default(public_vars.localize, 'late_recovery_pf_top_ratio', 1.12))) ...
    && (~getfield_with_default(q, 'gnss_valid', false) ...
        || (isfinite(q.disagreement_xy_m) && q.disagreement_xy_m <= getfield_with_default(public_vars.localize, 'late_recovery_disagree_m', 0.55))) ...
    && getfield_with_default(public_vars.localize, 'trace_last_jump_m', 0) <= getfield_with_default(public_vars.localize, 'late_recovery_est_jump_m', 0.20) ...
    && getfield_with_default(public_vars.localize, 'contradiction_counter', 0) <= getfield_with_default(public_vars.localize, 'late_recovery_max_contradictions', 1) ...
    && getfield_with_default(public_vars.localize, 'no_progress_counter', 0) <= getfield_with_default(public_vars.localize, 'late_recovery_max_no_progress', 2) ...
    && ~symmetry_hold_block ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && ~failed_commit_region_block ...
    && commit_separated;
near_goal_finish_ready = string(getfield_with_default(public_vars, 'nav_state', "localize")) == "relocalize" ...
    && isfinite(goal_dist_est) ...
    && goal_dist_est <= getfield_with_default(public_vars.localize, 'near_goal_finish_goal_dist_m', 0.95) ...
    && clear_steps >= getfield_with_default(public_vars.localize, 'near_goal_finish_clear_steps', 4) ...
    && isfinite(q.scan_match_cost) ...
    && q.scan_match_cost <= getfield_with_default(public_vars.localize, 'near_goal_finish_scan_cost', 0.08) ...
    && isfinite(q.pf_cluster_radius_m) ...
    && q.pf_cluster_radius_m <= getfield_with_default(public_vars.localize, 'near_goal_finish_pf_cluster_m', 0.16) ...
    && isfinite(q.pf_dominant_mass) ...
    && q.pf_dominant_mass >= getfield_with_default(public_vars.localize, 'near_goal_finish_pf_mass', 0.995) ...
    && isfinite(q.pf_top_ratio) ...
    && q.pf_top_ratio >= getfield_with_default(public_vars.localize, 'near_goal_finish_pf_top_ratio', 1.001) ...
    && ~symmetry_hold_block ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && ~failed_commit_region_block;

strong_pf = isfinite(q.pf_cluster_radius_m) ...
    && q.pf_cluster_radius_m <= public_vars.localize.pf_cluster_force_plan ...
    && isfinite(q.pf_dominant_mass) ...
    && q.pf_dominant_mass >= public_vars.localize.pf_dominant_mass_force_plan ...
    && pf_unique_force ...
    && isfinite(q.scan_match_cost) ...
    && q.scan_match_cost <= public_vars.localize.scan_cost_force_plan ...
    && (~isfinite(q.disagreement_xy_m) || q.disagreement_xy_m <= public_vars.localize.disagree_force_plan) ...
    && clear_ready ...
    && recovery_finish_motion_ready ...
    && ~symmetry_hold_block ...
    && informative_motion ...
    && ~side_swap_block ...
    && ~large_jump_block ...
    && ~failed_commit_region_block ...
    && commit_separated;

if strong_pf && ambiguity_ok
    tf = true;
    return;
end

if late_recovery_finish_ready && ambiguity_ok
    tf = true;
    return;
end

if near_goal_finish_ready && ambiguity_ok
    tf = true;
    return;
end

if isfinite(q.kf_var_xy_mean) && q.kf_var_xy_mean <= public_vars.localize.kf_var_ok ...
        && ambiguity_ok && fusion_consistent ...
        && (~isfield(q, 'lidar_valid') || ~q.lidar_valid || (pf_unique_ok && clear_ready)) ...
        && ~symmetry_hold_block ...
        && ~side_swap_block ...
        && ~large_jump_block ...
        && confirm_separated
    tf = true;
end
end

function tf = should_force_exploratory_commit(public_vars)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality) ...
        || ~isfield(public_vars, 'localization_commit') || isempty(public_vars.localization_commit)
    return;
end
q = public_vars.localization_quality;
lc = public_vars.localization_commit;
trace_bbox_diag = getfield_with_default(public_vars.localize, 'trace_bbox_diag_m', nan);
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
hypothesis_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
single_hypothesis = isfinite(hypothesis_count) && hypothesis_count <= 1;
ambiguity_active = ambiguity_active_or_ws(public_vars);
est_jump_m = getfield_with_default(public_vars.localize, 'trace_last_jump_m', 0);
failed_commit_pose = getfield_with_default(public_vars.localize, 'last_failed_commit_pose', [nan, nan, nan]);
failed_commit_region_block = false;
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2))) ...
        && numel(failed_commit_pose) >= 2 && all(isfinite(failed_commit_pose(1:2)))
    failed_commit_region_block = norm(public_vars.estimated_pose(1:2) - failed_commit_pose(1:2)) ...
        <= getfield_with_default(public_vars.localize, 'failed_commit_block_radius_m', 1.75);
end
tf = single_hypothesis ...
    && isfinite(trace_bbox_diag) && trace_bbox_diag >= lc.exploratory_commit_trace_bbox_m ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= max(lc.pf_cluster_single_commit_m, 0.22) ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= lc.exploratory_commit_mass ...
    && isfinite(q.scan_match_cost) && q.scan_match_cost <= lc.exploratory_commit_scan_cost ...
    && clear_steps >= lc.exploratory_commit_clear_steps ...
    && ~ambiguity_active ...
    && ~failed_commit_region_block ...
    && est_jump_m <= 0.25;
end

function tf = should_enter_safe_stop(public_vars)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
if ~q.pose_valid
    tf = true;
end
end

function tf = should_relocalize(public_vars)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
in_track_commit = isfield(public_vars, 'track') && isfield(public_vars.track, 'commit_counter') ...
    && public_vars.track.commit_counter > 0;
track_stall = 0;
scan_bad = false;
if isfield(public_vars, 'track')
    if isfield(public_vars.track, 'stall_steps') && isfinite(public_vars.track.stall_steps)
        track_stall = public_vars.track.stall_steps;
    end
    if isfield(public_vars.track, 'scan_mismatch_counter') && isfield(public_vars.track, 'scan_mismatch_steps')
        scan_bad = public_vars.track.scan_mismatch_counter >= public_vars.track.scan_mismatch_steps;
    end
end
if ~q.gnss_valid && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m > 1.05 ...
        && (track_stall >= 14 || scan_bad)
    tf = true;
    return;
end
if isfield(q, 'gnss_valid') && q.gnss_valid
    if isfinite(q.kf_var_xy_mean) && q.kf_var_xy_mean > 1.0
        tf = true;
        return;
    end
    if isfinite(q.disagreement_xy_m) && q.disagreement_xy_m > 1.25
        tf = true;
        return;
    end
end

if isfield(public_vars, 'track') && isfield(public_vars.track, 'force_relocalize') ...
        && public_vars.track.force_relocalize && ~in_track_commit ...
        && (scan_bad || track_stall >= 12)
    tf = true;
end
if in_track_commit && isfield(public_vars, 'track') ...
        && isfield(public_vars.track, 'scan_mismatch_counter') ...
        && public_vars.track.scan_mismatch_counter >= public_vars.track.scan_mismatch_steps
    tf = true;
end
end

function tf = should_enter_disambiguate(public_vars)
tf = false;
if isfield(public_vars, 'disambiguate') && isfield(public_vars.disambiguate, 'cooldown_until_step') ...
        && isfield(public_vars, 'global_step') && public_vars.global_step < public_vars.disambiguate.cooldown_until_step
    return;
end
if ~isfield(public_vars, 'environment_ambiguity') || isempty(public_vars.environment_ambiguity)
    return;
end
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
if any(commit_state == ["confirm", "committed"])
    return;
end
if is_localization_confident_for_plan(public_vars, true)
    return;
end
amb = public_vars.environment_ambiguity;
q = public_vars.localization_quality;
single_hypothesis_localized = isfinite(getfield_with_default(q, 'pf_hypothesis_count', nan)) ...
    && q.pf_hypothesis_count <= 1 ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.30 ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90 ...
    && (~isfield(q, 'scan_match_cost') || ~isfinite(q.scan_match_cost) || q.scan_match_cost <= public_vars.localize.scan_cost_ok);
if single_hypothesis_localized
    return;
end
commit_active = isfield(public_vars, 'startup_motion') && isfield(public_vars.startup_motion, 'commit_counter') ...
    && public_vars.startup_motion.commit_counter > 0;
front_explorable = isfield(public_vars, 'startup_motion') && isfield(public_vars.startup_motion, 'last_front_open') ...
    && isfinite(public_vars.startup_motion.last_front_open) ...
    && public_vars.startup_motion.last_front_open >= public_vars.startup_motion.min_commit_front_clear;
scan_informative = isfield(amb, 'scan_change') && isfinite(amb.scan_change) && amb.scan_change >= 0.14;
if commit_active && front_explorable && scan_informative
    return;
end

if isfield(public_vars, 'nav_state') && string(public_vars.nav_state) == "track"
    stable_track = isfield(q, 'path_distance_m') && isfinite(q.path_distance_m) ...
        && q.path_distance_m <= 0.25 ...
        && isfield(q, 'scan_match_cost') && isfinite(q.scan_match_cost) ...
        && q.scan_match_cost <= public_vars.localize.scan_cost_ok ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.30 ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.80;
    if stable_track
        return;
    end
    % During tracking, ambiguity alone should not interrupt stable progress.
    if isfield(public_vars, 'track') ...
            && isfield(public_vars.track, 'stall_steps') ...
            && public_vars.track.stall_steps < public_vars.disambiguate.stall_trigger_steps
        return;
    end
    poor_track = isfield(q, 'path_distance_m') && isfinite(q.path_distance_m) ...
        && q.path_distance_m > 0.60;
    poor_scan = isfield(q, 'scan_match_cost') && isfinite(q.scan_match_cost) ...
        && q.scan_match_cost > public_vars.localize.scan_cost_good;
    weak_pf = isfinite(q.pf_cluster_radius_m) && isfinite(q.pf_dominant_mass) ...
        && (q.pf_cluster_radius_m > 0.45 || q.pf_dominant_mass < 0.70);
    if ~(poor_track && poor_scan && weak_pf)
        return;
    end
end

if isfield(public_vars, 'nav_state') && string(public_vars.nav_state) == "relocalize"
    relocalize_stable = isfield(q, 'reason') && string(q.reason) == "ok" ...
        && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.35 ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.80 ...
        && (~isfield(q, 'scan_match_cost') || ~isfinite(q.scan_match_cost) ...
            || q.scan_match_cost <= public_vars.localize.scan_cost_ok);
    if relocalize_stable
        return;
    end
end

if ~amb.active || amb.counter < public_vars.ambiguity.required_steps
    return;
end
if ~isfinite(q.pf_cluster_radius_m) || ~isfinite(q.pf_dominant_mass)
    return;
end
if ~q.gnss_valid && amb.active ...
        && (q.pf_cluster_radius_m > 0.32 || q.pf_dominant_mass < 0.82 || q.pf_top_ratio < 1.12)
    tf = true;
    return;
end
if isfield(public_vars, 'track') ...
        && isfield(public_vars.track, 'stall_steps') ...
        && isfield(q, 'goal_distance_m') ...
        && public_vars.track.stall_steps >= public_vars.disambiguate.stall_trigger_steps ...
        && q.pf_dominant_mass > 0.82 ...
        && q.goal_distance_m <= public_vars.disambiguate.goal_est_trigger_m
    tf = true;
end
end

function gate = localization_navigation_gate_ws(public_vars)
gate = struct( ...
    'can_plan', false, ...
    'can_track', false, ...
    'can_info_track', false, ...
    'reason', "missing_quality", ...
    'recovery_state', "globalize");

if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
if ~isfield(public_vars, 'localization_commit') || isempty(public_vars.localization_commit)
    gate.reason = "missing_commit";
    return;
end
if isfield(public_vars, 'verify') && getfield_with_default(public_vars.verify, 'active', false) ...
        && getfield_with_default(public_vars.verify, 'navigation_allow_counter', 0) > 0
    gate.can_plan = true;
    gate.can_info_track = true;
    gate.reason = "verify_override";
    gate.recovery_state = "relocalize";
    return;
end

commit_state = string(get_nested_struct_or(public_vars, 'localization_commit', 'state', "search"));
if commit_state == "search"
    gate.reason = "commit_not_ready";
    return;
end
gate.recovery_state = "relocalize";

if commit_state == "committed"
    if ~is_localization_ready_to_execute_ws(public_vars)
        gate.reason = "execution_not_ready";
        return;
    end
    gate.can_plan = true;
    gate.can_track = true;
elseif commit_state == "confirm"
    if ~is_localization_ready_for_verify_ws(public_vars)
        gate.reason = "verify_not_ready";
        return;
    end
    gate.can_plan = true;
end
gate.can_info_track = isfield(public_vars, 'verify') ...
    && getfield_with_default(public_vars.verify, 'active', false);
if gate.can_info_track && getfield_with_default(public_vars.verify, 'navigation_allow_counter', 0) > 0
    gate.can_plan = true;
end
gate.reason = "ok";
end

function tf = is_localization_ready_for_verify_ws(public_vars)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
if ~getfield_with_default(q, 'pose_valid', false)
    return;
end
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
hyp_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
pf_unique = (isfinite(getfield_with_default(q, 'pf_top_ratio', nan)) ...
        && getfield_with_default(q, 'pf_top_ratio', nan) >= 1.02) ...
    || (isfinite(hyp_count) && hyp_count <= 1);
tf = isfinite(getfield_with_default(q, 'pf_cluster_radius_m', nan)) ...
    && getfield_with_default(q, 'pf_cluster_radius_m', nan) <= 0.28 ...
    && isfinite(getfield_with_default(q, 'pf_dominant_mass', nan)) ...
    && getfield_with_default(q, 'pf_dominant_mass', nan) >= 0.88 ...
    && pf_unique ...
    && isfinite(getfield_with_default(q, 'scan_match_cost', nan)) ...
    && getfield_with_default(q, 'scan_match_cost', nan) <= getfield_with_default(public_vars.localize, 'scan_cost_ok', 0.22) ...
    && clear_steps >= max(2, getfield_with_default(public_vars.localize, 'finish_clear_steps_min', 2)) ...
    && getfield_with_default(public_vars.localize, 'contradiction_counter', 0) <= 2 ...
    && getfield_with_default(public_vars.localize, 'no_progress_counter', 0) <= 3;
end

function tf = is_localization_ready_to_execute_ws(public_vars)
tf = false;
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    return;
end
q = public_vars.localization_quality;
if ~getfield_with_default(q, 'pose_valid', false)
    return;
end
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
hyp_count = getfield_with_default(q, 'pf_hypothesis_count', nan);
pf_unique = (isfinite(getfield_with_default(q, 'pf_top_ratio', nan)) ...
        && getfield_with_default(q, 'pf_top_ratio', nan) >= 1.01) ...
    || (isfinite(hyp_count) && hyp_count <= 1);
tf = isfinite(getfield_with_default(q, 'pf_cluster_radius_m', nan)) ...
    && getfield_with_default(q, 'pf_cluster_radius_m', nan) <= 0.22 ...
    && isfinite(getfield_with_default(q, 'pf_dominant_mass', nan)) ...
    && getfield_with_default(q, 'pf_dominant_mass', nan) >= 0.96 ...
    && pf_unique ...
    && isfinite(getfield_with_default(q, 'scan_match_cost', nan)) ...
    && getfield_with_default(q, 'scan_match_cost', nan) <= getfield_with_default(public_vars.localize, 'scan_cost_ok', 0.22) ...
    && clear_steps >= max(2, getfield_with_default(public_vars.localize, 'finish_clear_steps_min', 2)) ...
    && getfield_with_default(public_vars.localize, 'contradiction_counter', 0) <= 2 ...
    && getfield_with_default(public_vars.localize, 'no_progress_counter', 0) <= 3 ...
    && getfield_with_default(public_vars.localize, 'large_jump_hold_counter', 0) <= 0 ...
    && getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0) <= 0;
end

function tf = should_replan(public_vars)
tf = false;
if isfield(public_vars, 'track') && isfield(public_vars.track, 'force_replan')
    tf = public_vars.track.force_replan;
end
end

function public_vars = localization_exploration_motion(read_only_vars, public_vars)
public_vars.localize.motion_step = public_vars.localize.motion_step + 1;
L = read_only_vars.agent_drive.interwheel_dist;

[front, left, right] = lidar_sector_minima(read_only_vars);
mode = "localize";

[use_candidate_motion, v, w, startup_info] = choose_informative_localization_action(read_only_vars, public_vars, mode, front, left, right);
if use_candidate_motion
    public_vars.startup_motion.last_mode = startup_info.mode;
    public_vars.startup_motion.last_best_heading = startup_info.best_heading;
    public_vars.startup_motion.last_left_open = startup_info.left_open;
    public_vars.startup_motion.last_right_open = startup_info.right_open;
    public_vars.startup_motion.last_front_open = startup_info.front_open;
else

    [use_sector_motion, v, w, startup_info] = startup_sector_motion(read_only_vars, public_vars, mode);
    public_vars.startup_motion.last_mode = startup_info.mode;
    public_vars.startup_motion.last_best_heading = startup_info.best_heading;
    public_vars.startup_motion.last_left_open = startup_info.left_open;
    public_vars.startup_motion.last_right_open = startup_info.right_open;
    public_vars.startup_motion.last_front_open = startup_info.front_open;
    if ~use_sector_motion
    [use_wall_follow, wall_side, align_counter, v_wf, w_wf] = wall_follow_command( ...
        read_only_vars, front, left, right, public_vars, public_vars.localize.wall_side, public_vars.localize.wall_align_counter, mode);
    public_vars.localize.wall_side = wall_side;
    public_vars.localize.wall_align_counter = align_counter;

    if use_wall_follow
        v = v_wf;
        w = w_wf;
    elseif front > 0.75
        v = 0.09;
        if left >= right
            w = public_vars.localize.turn_rate;
        else
            w = -public_vars.localize.turn_rate;
        end
    else
        v = 0;
        public_vars.localize.wall_side = "none";
        public_vars.localize.wall_align_counter = 0;
        if left >= right
            w = public_vars.localize.turn_rate_blocked;
        else
            w = -public_vars.localize.turn_rate_blocked;
        end
    end
    else
    public_vars.localize.wall_side = "none";
    public_vars.localize.wall_align_counter = 0;
    if abs(w) < 0.08 && isfinite(front) && front < 1.05
        v = min(v, 0.06);
        if right > left
            w = -max(public_vars.localize.turn_rate, 0.28);
        else
            w = max(public_vars.localize.turn_rate, 0.28);
        end
    end
end
end

[v, w] = apply_map_edge_guard_ws(read_only_vars, public_vars, v, w, mode);
public_vars.localize.cmd_distance_accum = public_vars.localize.cmd_distance_accum + abs(v) * read_only_vars.sampling_period;
public_vars.localize.cmd_turn_accum = public_vars.localize.cmd_turn_accum + abs(w) * read_only_vars.sampling_period;
public_vars.motion_vector = sanitize_motion_vector_ws([v + 0.5 * L * w, v - 0.5 * L * w]);
end

function public_vars = disambiguation_motion(read_only_vars, public_vars)
L = read_only_vars.agent_drive.interwheel_dist;
[front, left, right] = lidar_sector_minima(read_only_vars);
mode = "disambiguate";

[use_candidate_motion, v, w, startup_info] = choose_informative_localization_action(read_only_vars, public_vars, mode, front, left, right);
if use_candidate_motion
    public_vars.startup_motion.last_mode = startup_info.mode;
    public_vars.startup_motion.last_best_heading = startup_info.best_heading;
    public_vars.startup_motion.last_left_open = startup_info.left_open;
    public_vars.startup_motion.last_right_open = startup_info.right_open;
    public_vars.startup_motion.last_front_open = startup_info.front_open;
else
    [use_sector_motion, v, w, startup_info] = startup_sector_motion(read_only_vars, public_vars, mode);
    public_vars.startup_motion.last_mode = startup_info.mode;
    public_vars.startup_motion.last_best_heading = startup_info.best_heading;
    public_vars.startup_motion.last_left_open = startup_info.left_open;
    public_vars.startup_motion.last_right_open = startup_info.right_open;
    public_vars.startup_motion.last_front_open = startup_info.front_open;
    if ~use_sector_motion
    [use_wall_follow, wall_side, align_counter, v_wf, w_wf] = wall_follow_command( ...
        read_only_vars, front, left, right, public_vars, public_vars.disambiguate.wall_side, public_vars.disambiguate.wall_align_counter, mode);
    public_vars.disambiguate.wall_side = wall_side;
    public_vars.disambiguate.wall_align_counter = align_counter;

    if use_wall_follow
        v = v_wf;
        w = w_wf;
    else
        v = 0;
        public_vars.disambiguate.wall_side = "none";
        public_vars.disambiguate.wall_align_counter = 0;
        if left >= right
            w = public_vars.disambiguate.turn_rate;
        else
            w = -public_vars.disambiguate.turn_rate;
        end
    end
    else
    public_vars.disambiguate.wall_side = "none";
    public_vars.disambiguate.wall_align_counter = 0;
    if abs(w) < 0.08 && isfinite(front) && front < 1.10
        v = min(v, 0.07);
        if right > left
            w = -max(public_vars.disambiguate.turn_rate, 0.38);
        else
            w = max(public_vars.disambiguate.turn_rate, 0.38);
        end
    end
end
end

[v, w] = apply_map_edge_guard_ws(read_only_vars, public_vars, v, w, mode);
public_vars.localize.cmd_distance_accum = public_vars.localize.cmd_distance_accum + abs(v) * read_only_vars.sampling_period;
public_vars.localize.cmd_turn_accum = public_vars.localize.cmd_turn_accum + abs(w) * read_only_vars.sampling_period;
public_vars.motion_vector = sanitize_motion_vector_ws([v + 0.5 * L * w, v - 0.5 * L * w]);
end

function [use_motion, v, w, info] = choose_informative_localization_action(read_only_vars, public_vars, mode, front, left, right)
use_motion = false;
v = 0;
w = 0;
info = struct('mode', "candidate", 'best_heading', nan, 'best_score', -inf, ...
    'left_open', left, 'right_open', right, 'front_open', front);

[hypotheses, hyp_weights] = top_localization_hypotheses(public_vars, 3);
if size(hypotheses, 1) < 2 || ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars, 'lidar_config')
    return;
end

L = read_only_vars.agent_drive.interwheel_dist;
candidate_defs = [ ...
    0.00,  0.00,  0.55; ...
    0.00,  0.00, -0.55; ...
    0.11,  0.18,  0.00; ...
    0.14,  0.30,  0.00; ...
    0.08,  0.20,  0.45; ...
    0.08,  0.20, -0.45; ...
    0.12,  0.30,  0.32; ...
    0.12,  0.30, -0.32; ...
    0.16,  0.42,  0.20; ...
    0.16,  0.42, -0.20];
if mode == "disambiguate"
    candidate_defs(3, 1:2) = [0.15, 0.28];
    candidate_defs(4, 1:2) = [0.18, 0.40];
    candidate_defs(5:6, 1:2) = repmat([0.12, 0.28], 2, 1);
    candidate_defs(7:8, 1:2) = repmat([0.16, 0.38], 2, 1);
    candidate_defs(9:10, 1:2) = repmat([0.20, 0.52], 2, 1);
end

best_score = -inf;
best_idx = 0;
for i = 1:size(candidate_defs, 1)
    v_try = candidate_defs(i, 1);
    step_len = candidate_defs(i, 2);
    w_try = candidate_defs(i, 3);
    score = score_localization_action(read_only_vars, public_vars, hypotheses, hyp_weights, v_try, w_try, step_len, front, left, right, L);
    if score > best_score
        best_score = score;
        best_idx = i;
    end
end

goal_score = -inf;
goal_motion = [0, 0];
goal_heading = nan;
[goal_score, goal_motion, goal_heading] = score_informative_goal_action( ...
    read_only_vars, public_vars, hypotheses, hyp_weights, mode, front, left, right);

if goal_score > best_score + 0.06
    v = goal_motion(1);
    w = goal_motion(2);
    use_motion = true;
    info.mode = "goal_candidate";
    info.best_score = goal_score;
    info.best_heading = goal_heading;
    return;
end

if best_idx == 0 || best_score < 0.10
    return;
end
v = candidate_defs(best_idx, 1);
w = candidate_defs(best_idx, 3);
use_motion = true;
info.best_score = best_score;
info.best_heading = wrap_to_pi_ws(getfield_with_default(public_vars.startup_motion, 'last_best_heading', 0) + 0.30 * w);
end

function [best_score, best_motion, best_heading] = score_informative_goal_action(read_only_vars, public_vars, hypotheses, hyp_weights, mode, front, left, right)
best_score = -inf;
best_motion = [0, 0];
best_heading = nan;

if ~isfield(public_vars, 'estimated_pose') || numel(public_vars.estimated_pose) < 3 ...
        || any(~isfinite(public_vars.estimated_pose(1:3)))
    return;
end
if ~isfield(read_only_vars, 'discrete_map') || isempty(read_only_vars.discrete_map)
    return;
end

pose = public_vars.estimated_pose(1:3);
heading_offsets = [-85, -55, -30, 30, 55, 85] * pi / 180;
goal_dists = [0.9, 1.4];
if mode == "disambiguate"
    heading_offsets = [-95, -65, -35, 35, 65, 95] * pi / 180;
    goal_dists = [1.1, 1.8];
end

for i = 1:numel(goal_dists)
    for j = 1:numel(heading_offsets)
        heading = wrap_to_pi_ws(pose(3) + heading_offsets(j));
        goal_xy = pose(1:2) + goal_dists(i) * [cos(heading), sin(heading)];
        if ~is_short_goal_feasible_ws(goal_xy, read_only_vars, public_vars)
            continue;
        end

        [path, path_len] = short_goal_path_ws(goal_xy, read_only_vars, public_vars);
        if isempty(path) || size(path, 1) < 2 || ~isfinite(path_len)
            continue;
        end

        preview_dist = min(goal_dists(i), ternary_ws(mode == "disambiguate", 1.0, 0.8));
        preview_xy = preview_path_point_ws(path, preview_dist);
        if any(~isfinite(preview_xy))
            continue;
        end

        desired_heading = atan2(preview_xy(2) - pose(2), preview_xy(1) - pose(1));
        heading_err = wrap_to_pi_ws(desired_heading - pose(3));
        step_len = norm(preview_xy - pose(1:2));
        if step_len < 0.20
            continue;
        end

        v_try = min(ternary_ws(mode == "disambiguate", 0.24, 0.20), max(0.10, 0.42 * step_len));
        w_limit = ternary_ws(mode == "disambiguate", 0.90, 0.72);
        w_try = max(min(heading_err / max(read_only_vars.sampling_period, 1e-3), w_limit), -w_limit);

        score = score_localization_action(read_only_vars, public_vars, hypotheses, hyp_weights, ...
            v_try, w_try, step_len, front, left, right, read_only_vars.agent_drive.interwheel_dist);
        if ~isfinite(score)
            continue;
        end

        direct_len = norm(goal_xy - pose(1:2));
        length_ratio = path_len / max(direct_len, 1e-6);
        score = score + 0.16 * min(step_len, 1.3) - 0.10 * max(0, length_ratio - 1.25);
        if mode == "disambiguate"
            score = score + 0.08 * goal_dists(i);
        end

        if score > best_score
            best_score = score;
            best_motion = [v_try, w_try];
            best_heading = desired_heading;
        end
    end
end
end

function [v, w] = apply_map_edge_guard_ws(read_only_vars, public_vars, v, w, mode)
if ~isfield(read_only_vars, 'discrete_map') || ~isfield(read_only_vars.discrete_map, 'limits') ...
        || numel(read_only_vars.discrete_map.limits) < 4
    return;
end
xy = [nan, nan];
theta = nan;
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2 ...
        && all(isfinite(read_only_vars.gnss_position(1:2)))
    xy = read_only_vars.gnss_position(1:2);
end
if isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2)))
    if any(~isfinite(xy))
        xy = public_vars.estimated_pose(1:2);
    end
    if numel(public_vars.estimated_pose) >= 3 && isfinite(public_vars.estimated_pose(3))
        theta = public_vars.estimated_pose(3);
    end
end
if any(~isfinite(xy))
    return;
end
limits = read_only_vars.discrete_map.limits(:)';
edge_d = [xy(1) - limits(1), limits(3) - xy(1), xy(2) - limits(2), limits(4) - xy(2)];
min_edge = min(edge_d);
soft_margin = 1.00;
hard_margin = 0.70;
if min_edge >= soft_margin
    return;
end
if min_edge < hard_margin
    v = 0;
else
    v = min(v, 0.02 + 0.08 * max(0, min(1, (min_edge - hard_margin) / max(soft_margin - hard_margin, 1e-6))));
end
if isfinite(theta)
    center_xy = [(limits(1) + limits(3)) / 2, (limits(2) + limits(4)) / 2];
    inward_heading = atan2(center_xy(2) - xy(2), center_xy(1) - xy(1));
    inward_rel = wrap_to_pi_ws(inward_heading - theta);
    guard_w = 0.40;
    if mode == "disambiguate"
        guard_w = 0.50;
    end
    if abs(inward_rel) > 8 * pi / 180
        w = sign(inward_rel) * max(abs(w), guard_w);
    end
end
end

function score = score_localization_action(read_only_vars, public_vars, hypotheses, hyp_weights, v_try, w_try, step_len, front, left, right, L)
score = -inf;
if isempty(hypotheses)
    return;
end
pose_delta = [step_len, 0, w_try * read_only_vars.sampling_period];
pred = cell(size(hypotheses, 1), 1);
clear_terms = nan(size(hypotheses, 1), 1);
for i = 1:size(hypotheses, 1)
    pose = hypotheses(i, :);
    pose(1) = pose(1) + pose_delta(1) * cos(pose(3));
    pose(2) = pose(2) + pose_delta(1) * sin(pose(3));
    pose(3) = wrap_to_pi_ws(pose(3) + pose_delta(3));
    pred{i} = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
    clear_terms(i) = local_pose_clearance_score_ws(pose, read_only_vars, public_vars);
end

sep = 0;
for i = 1:numel(pred)
    for j = (i + 1):numel(pred)
        vi = pred{i};
        vj = pred{j};
        valid = isfinite(vi) & isfinite(vj);
        if any(valid)
            sep = sep + hyp_weights(i) * hyp_weights(j) * mean(abs(vi(valid) - vj(valid)));
        end
    end
end

safety = 1.0;
if v_try > 0 && isfinite(front)
    safety = min(safety, max(0, min(1, (front - 0.35) / 0.90)));
end
if w_try > 0 && isfinite(left)
    safety = min(safety, max(0, min(1, (left - 0.20) / 0.80)));
elseif w_try < 0 && isfinite(right)
    safety = min(safety, max(0, min(1, (right - 0.20) / 0.80)));
end
if safety <= 0.05
    score = -inf;
    return;
end

future_heading = w_try * read_only_vars.sampling_period;
sector_id = heading_sector_id_ws(future_heading, getfield_with_default(public_vars.disambiguate, 'sector_width_rad', 45 * pi / 180));
sector_hits = sum(getfield_with_default(public_vars.disambiguate, 'visited_sectors', []) == sector_id);
signature = corridor_signature_ws(front, left, right, future_heading);
visited_signatures = getfield_with_default(public_vars.disambiguate, 'visited_signatures', strings(0, 1));
signature_hits = sum(visited_signatures == signature);

motion_balance = 0.10 * abs(w_try) + 0.28 * v_try + 0.08 * step_len;
revisit_penalty = 0.22 * sector_hits + 0.18 * signature_hits;
clearance_term = weighted_mean_ws(clear_terms, hyp_weights);
clearance_gain = getfield_with_default(public_vars.disambiguate, 'clearance_action_gain', 0.55);
score = 1.28 * sep + motion_balance + 0.24 * safety + clearance_gain * clearance_term - revisit_penalty;
end

function tf = is_short_goal_feasible_ws(goal_xy, read_only_vars, public_vars)
tf = false;
if numel(goal_xy) < 2 || any(~isfinite(goal_xy(1:2)))
    return;
end
if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'limits')
    return;
end
limits = read_only_vars.map.limits;
if goal_xy(1) <= limits(1) || goal_xy(1) >= limits(3) || goal_xy(2) <= limits(2) || goal_xy(2) >= limits(4)
    return;
end
clearance = getfield_with_default(public_vars, 'tracking_clearance_m', 0.25);
tf = ~is_collision_with_walls_ws(goal_xy, read_only_vars.map.walls, max(0.12, 0.85 * clearance));
end

function [path, path_len] = short_goal_path_ws(goal_xy, read_only_vars, public_vars)
path = [];
path_len = nan;
tmp_ro = read_only_vars;
tmp_ro.map.goal = goal_xy(:)';
tmp_pub = public_vars;
tmp_pub.path = [];
tmp_pub.raw_path = [];
tmp_pub.smoothed_path = [];
tmp_pub.replan_path = true;
path = plan_path(tmp_ro, tmp_pub);
if isempty(path) || size(path, 1) < 2
    return;
end
path_len = sum(vecnorm(diff(path(:, 1:2), 1, 1), 2, 2));
end

function xy = preview_path_point_ws(path, dist_limit)
xy = [nan, nan];
if isempty(path) || size(path, 1) < 2
    return;
end
remain = max(dist_limit, 0);
xy = path(1, 1:2);
for i = 2:size(path, 1)
    seg = path(i, 1:2) - path(i - 1, 1:2);
    seg_len = norm(seg);
    if seg_len <= 1e-9
        continue;
    end
    if remain <= seg_len
        xy = path(i - 1, 1:2) + (remain / seg_len) * seg;
        return;
    end
    remain = remain - seg_len;
    xy = path(i, 1:2);
end
end

function out = ternary_ws(condition, true_value, false_value)
if condition
    out = true_value;
else
    out = false_value;
end
end

function tf = is_collision_with_walls_ws(point, walls, tol)
tf = false;
if isempty(walls)
    return;
end
for i = 1:size(walls, 1)
    a = walls(i, 1:2);
    b = walls(i, 3:4);
    ab = b - a;
    if all(abs(ab) < 1e-12)
        d = norm(point - a);
    else
        t = dot(point - a, ab) / max(dot(ab, ab), 1e-12);
        t = max(0, min(1, t));
        p = a + t * ab;
        d = norm(point - p);
    end
    if d <= tol
        tf = true;
        return;
    end
end
end

function signature = corridor_signature_ws(front, left, right, heading)
front_bin = discretize_metric_ws(front, [0.8, 1.6, 3.0]);
left_bin = discretize_metric_ws(left, [0.5, 1.0, 2.0]);
right_bin = discretize_metric_ws(right, [0.5, 1.0, 2.0]);
sector_bin = heading_sector_id_ws(heading, 45 * pi / 180);
signature = sprintf('%d-%d-%d-%d', front_bin, left_bin, right_bin, sector_bin);
end

function idx = discretize_metric_ws(value, cuts)
if ~isfinite(value)
    idx = numel(cuts) + 1;
    return;
end
idx = find(value <= cuts, 1, 'first');
if isempty(idx)
    idx = numel(cuts) + 1;
end
end

function [use_motion, v, w, info] = startup_sector_motion(read_only_vars, public_vars, mode)
use_motion = false;
v = 0;
w = 0;
info = struct('mode', "unknown", 'best_heading', nan, 'best_score', -inf, ...
    'left_open', inf, 'right_open', inf, 'front_open', inf);

if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || ~isfield(read_only_vars, 'lidar_config') || isempty(read_only_vars.lidar_config)
    return;
end

d = read_only_vars.lidar_distances(:);
a = wrap_to_pi_ws(read_only_vars.lidar_config(:));
valid = isfinite(d) & isfinite(a);
d = d(valid);
a = a(valid);
if numel(d) < 5
    return;
end

front_mask = abs(a) <= 22 * pi / 180;
if any(front_mask)
    front_min = min(d(front_mask));
else
    front_min = min(d);
end
left_close_mask = a >= 25 * pi / 180 & a <= 115 * pi / 180;
right_close_mask = a <= -25 * pi / 180 & a >= -115 * pi / 180;
if any(left_close_mask)
    left_close_min = min(d(left_close_mask));
else
    left_close_min = inf;
end
if any(right_close_mask)
    right_close_min = min(d(right_close_mask));
else
    right_close_min = inf;
end

cfg = public_vars.startup_motion;
clip_d = min(d, cfg.open_beam_clip_m);
best_score = -inf;
best_heading = 0;
for i = 1:numel(a)
    mask = abs(wrap_to_pi_ws(a - a(i))) <= cfg.sector_half_angle_rad;
    if nnz(mask) < 3
        continue;
    end
    sector_vals = clip_d(mask);
    score = 0.55 * median(sector_vals) + 0.45 * max(sector_vals) - 0.06 * std(sector_vals);
    score = score + informative_heading_bonus(read_only_vars, public_vars, a(i), mode);
    if abs(a(i)) < cfg.target_sector_min_angle_rad
        score = score - 0.22;
    end
    if score > best_score
        best_score = score;
        best_heading = a(i);
    end
end

left_open = directional_open_metric(d, a, 80 * pi / 180, 30 * pi / 180, cfg.open_beam_clip_m);
right_open = directional_open_metric(d, a, -80 * pi / 180, 30 * pi / 180, cfg.open_beam_clip_m);
front_open = directional_open_metric(d, a, 0, 25 * pi / 180, cfg.open_beam_clip_m);
near_fraction = mean(d <= cfg.closed_front_max_m);
closed_mode = isfinite(left_open) && isfinite(right_open) ...
    && left_open <= cfg.closed_side_max_m && right_open <= cfg.closed_side_max_m ...
    && (front_open <= cfg.closed_front_max_m || near_fraction >= cfg.closed_near_fraction);

if mode == "disambiguate"
    heading_gain = cfg.heading_gain_disambiguate;
    center_gain = cfg.center_gain_disambiguate;
    soft_center_gain = cfg.soft_center_gain_disambiguate;
    w_max = cfg.w_max_disambiguate;
    v_open = cfg.v_open_disambiguate;
    v_closed = cfg.v_closed_disambiguate;
    heading_smooth_alpha = cfg.heading_smooth_alpha_disambiguate;
    max_heading_step = cfg.max_heading_step_disambiguate;
    step_idx = max(1, public_vars.nav_state_step);
else
    heading_gain = cfg.heading_gain_localize;
    center_gain = cfg.center_gain_localize;
    soft_center_gain = cfg.soft_center_gain_localize;
    w_max = cfg.w_max_localize;
    v_open = cfg.v_open_localize;
    v_closed = cfg.v_closed_localize;
    heading_smooth_alpha = cfg.heading_smooth_alpha_localize;
    max_heading_step = cfg.max_heading_step_localize;
    step_idx = max(1, public_vars.localize.motion_step);
end

scan_change = inf;
if isfield(public_vars, 'environment_ambiguity') && isfield(public_vars.environment_ambiguity, 'scan_change') ...
        && isfinite(public_vars.environment_ambiguity.scan_change)
    scan_change = public_vars.environment_ambiguity.scan_change;
end
info_gain_mode = any(string(public_vars.nav_state) == ["relocalize", "disambiguate"]) ...
    && ((isfinite(scan_change) && scan_change <= cfg.info_gain_scan_change) ...
        || getfield_with_default(public_vars.localize, 'info_gain_recovery_counter', 0) > 0);
if info_gain_mode
    heading_gain = heading_gain * cfg.info_gain_heading_scale;
    w_max = w_max * cfg.info_gain_wmax_scale;
    v_open = v_open * cfg.info_gain_v_scale;
    v_closed = v_closed * cfg.info_gain_v_scale;
end

center_term = 0;
if isfinite(left_open) && isfinite(right_open)
    center_term = soft_center_gain * atan2(left_open - right_open, 0.60);
end
if closed_mode && isfinite(left_open) && isfinite(right_open)
    center_term = center_gain * atan2(left_open - right_open, 0.45);
elseif isfinite(front_open) && front_open > 1.4
    center_term = 0.55 * center_term;
end

target_heading = wrap_to_pi_ws(best_heading);
turn_bias = sign(left_open - right_open);
if turn_bias == 0 || ~isfinite(turn_bias)
    turn_bias = sign(best_heading);
end
if turn_bias == 0 || ~isfinite(turn_bias)
    turn_bias = 1;
end
if closed_mode
    info.mode = "closed";
    target_heading = wrap_to_pi_ws(target_heading + cfg.closed_bias_gain * center_term);
    if isfinite(front_open) && front_open < 1.0
        target_heading = wrap_to_pi_ws(target_heading + 0.35 * sign(left_open - right_open));
    end
    if isfinite(front_min) && front_min <= cfg.closed_front_preturn_m
        target_heading = turn_bias * max(abs(target_heading), cfg.closed_preturn_heading_rad);
    end
    sweep = cfg.closed_sweep_gain * sin(2 * pi * step_idx / max(cfg.closed_sweep_period_steps, 2));
else
    info.mode = "open";
    sweep = 0;
end

[edge_heading_bias, edge_speed_scale, edge_active] = startup_edge_recovery_ws(read_only_vars, public_vars);
if edge_active
    target_heading = wrap_to_pi_ws(target_heading + cfg.edge_heading_gain * edge_heading_bias);
end

if isfield(public_vars, 'startup_motion') && isfinite(public_vars.startup_motion.filtered_heading_rad)
    prev_heading = public_vars.startup_motion.filtered_heading_rad;
    delta_heading = wrap_to_pi_ws(target_heading - prev_heading);
    delta_heading = max(min(delta_heading, max_heading_step), -max_heading_step);
    target_heading = wrap_to_pi_ws(prev_heading + heading_smooth_alpha * delta_heading);
end
public_vars.startup_motion.filtered_heading_rad = target_heading;

info.best_heading = target_heading;
info.best_score = best_score;
info.left_open = left_open;
info.right_open = right_open;
info.front_open = front_open;

commit_steps = cfg.commit_steps_localize;
if mode == "disambiguate"
    commit_steps = cfg.commit_steps_disambiguate;
end
if isfield(public_vars, 'startup_motion') ...
        && isfinite(public_vars.startup_motion.commit_heading_rad) ...
        && public_vars.startup_motion.commit_counter > 0
    committed_heading = public_vars.startup_motion.commit_heading_rad;
    committed_score = public_vars.startup_motion.commit_score;
    reverse_turn = abs(wrap_to_pi_ws(target_heading - committed_heading)) >= cfg.commit_forbid_reverse_rad;
    weak_improvement = ~isfinite(best_score) || best_score < committed_score + cfg.commit_release_better_score;
    if (reverse_turn && weak_improvement) || (isfinite(scan_change) && scan_change < cfg.commit_release_scan_change && weak_improvement)
        target_heading = committed_heading;
        best_score = committed_score;
        info.best_heading = target_heading;
        info.best_score = best_score;
    else
        public_vars.startup_motion.commit_heading_rad = target_heading;
        public_vars.startup_motion.commit_score = best_score;
        public_vars.startup_motion.commit_counter = commit_steps;
    end
else
    public_vars.startup_motion.commit_heading_rad = target_heading;
    public_vars.startup_motion.commit_score = best_score;
    public_vars.startup_motion.commit_counter = commit_steps;
end

turn_in_place_angle = cfg.turn_in_place_angle_rad;
slow_turn_angle = cfg.slow_turn_angle_rad;
if info_gain_mode
    turn_in_place_angle = min(turn_in_place_angle, cfg.info_gain_turn_in_place_angle_rad);
    slow_turn_angle = min(slow_turn_angle, cfg.info_gain_slow_turn_angle_rad);
end

if abs(target_heading) >= turn_in_place_angle
    v = 0;
elseif abs(target_heading) >= slow_turn_angle
    v = 0.06;
else
    if closed_mode
        v = v_closed;
    else
        v = v_open;
    end
    if isfinite(front_open)
        v = min(v, max(0.06, 0.12 * front_open));
    end
end
if edge_active
    v = max(0, v * edge_speed_scale);
    if abs(edge_heading_bias) >= cfg.edge_turn_only_heading_rad
        v = 0;
    end
end

w = heading_gain * target_heading + center_term + sweep;
w = max(min(w, w_max), -w_max);
if closed_mode && abs(w) < cfg.closed_min_turn_rad_s
    turn_sign = sign(left_open - right_open);
    if turn_sign == 0 || ~isfinite(turn_sign)
        turn_sign = sign(sweep);
    end
    if turn_sign == 0
        turn_sign = 1;
    end
    w = turn_sign * cfg.closed_min_turn_rad_s;
end
if abs(target_heading) >= cfg.best_sector_turn_only_rad
    v = 0;
end

front_turn_w = cfg.closed_turn_in_place_w_localize;
front_arc_v = cfg.closed_arc_v_localize;
if mode == "disambiguate"
    front_turn_w = cfg.closed_turn_in_place_w_disambiguate;
    front_arc_v = cfg.closed_arc_v_disambiguate;
end
if closed_mode && isfinite(front_min)
    if front_min <= cfg.closed_front_hard_turn_m
        v = 0;
        w = turn_bias * max(abs(w), front_turn_w);
    elseif front_min <= cfg.closed_front_turn_only_m
        v = 0;
        w = turn_bias * max(abs(w), 0.90 * front_turn_w);
    elseif front_min <= cfg.closed_front_arc_limit_m
        v = min(v, front_arc_v);
        w = turn_bias * max(abs(w), max(cfg.closed_min_turn_rad_s, 0.82 * front_turn_w));
    elseif front_min <= cfg.closed_front_preturn_m
        v = min(v, 0.05);
        w = turn_bias * max(abs(w), max(cfg.closed_min_turn_rad_s, 0.70 * front_turn_w));
    end
end

if isfinite(front_min)
    obstacle_turn_sign = sign(left_close_min - right_close_min);
    if obstacle_turn_sign == 0 || ~isfinite(obstacle_turn_sign)
        obstacle_turn_sign = turn_bias;
    end
    if obstacle_turn_sign == 0 || ~isfinite(obstacle_turn_sign)
        obstacle_turn_sign = 1;
    end
    if front_min <= 0.45
        v = 0;
        w = obstacle_turn_sign * max(abs(w), front_turn_w);
    elseif front_min <= 0.70
        v = 0;
        w = obstacle_turn_sign * max(abs(w), 0.90 * front_turn_w);
    elseif front_min <= 1.00
        v = min(v, front_arc_v);
        w = obstacle_turn_sign * max(abs(w), 0.75 * front_turn_w);
    end
end

side_escape_turn = cfg.side_escape_turn_localize;
if mode == "disambiguate"
    side_escape_turn = cfg.side_escape_turn_disambiguate;
end
if isfinite(left_open) && left_open < cfg.side_escape_dist_m
    v = min(v, 0.00);
    w = -max(abs(w), side_escape_turn);
elseif isfinite(right_open) && right_open < cfg.side_escape_dist_m
    v = min(v, 0.00);
    w = max(abs(w), side_escape_turn);
end
if closed_mode && abs(w) > 0.32
    v = min(v, 0.09);
end
q_loc = getfield_with_default(public_vars, 'localization_quality', struct());
reason_loc = string(getfield_with_default(q_loc, 'reason', "ok"));
disagree_loc = getfield_with_default(q_loc, 'disagreement_xy_m', nan);
pf_mass_loc = getfield_with_default(q_loc, 'pf_dominant_mass', nan);
gnss_valid_loc = getfield_with_default(q_loc, 'gnss_valid', false);
rotate_only = ~closed_mode ...
    && gnss_valid_loc ...
    && isfinite(front_open) && front_open >= cfg.rotate_only_front_clear_m ...
    && reason_loc == "fusion_disagreement_large" ...
    && ((isfinite(disagree_loc) && disagree_loc >= cfg.rotate_only_disagree_m) ...
        || (isfinite(pf_mass_loc) && pf_mass_loc <= cfg.rotate_only_pf_mass));
if rotate_only
    v = 0;
    turn_sign = sign(target_heading);
    if turn_sign == 0 || ~isfinite(turn_sign)
        turn_sign = sign(edge_heading_bias);
    end
    if turn_sign == 0 || ~isfinite(turn_sign)
        turn_sign = turn_bias;
    end
    if turn_sign == 0 || ~isfinite(turn_sign)
        turn_sign = 1;
    end
    w = turn_sign * max(abs(w), front_turn_w);
end
public_vars.startup_motion.commit_counter = max(0, public_vars.startup_motion.commit_counter - 1);
use_motion = true;
end

function [heading_bias, speed_scale, active] = startup_edge_recovery_ws(read_only_vars, public_vars)
heading_bias = 0;
speed_scale = 1.0;
active = false;
if ~isfield(read_only_vars, 'discrete_map') || ~isfield(read_only_vars.discrete_map, 'limits') ...
        || numel(read_only_vars.discrete_map.limits) < 4
    return;
end
cfg = getfield_with_default(public_vars, 'startup_motion', struct());
soft_margin = getfield_with_default(cfg, 'edge_margin_m', 1.20);
hard_margin = min(soft_margin - 0.05, getfield_with_default(cfg, 'edge_hard_margin_m', 0.55));
if ~isfinite(soft_margin) || soft_margin <= 0
    return;
end
hard_margin = max(0.05, hard_margin);
pose_xy = [nan, nan];
pose_theta = nan;
if isfield(read_only_vars, 'gnss_position') && numel(read_only_vars.gnss_position) >= 2 ...
        && all(isfinite(read_only_vars.gnss_position(1:2)))
    pose_xy = read_only_vars.gnss_position(1:2);
    if numel(read_only_vars.gnss_position) >= 3 && isfinite(read_only_vars.gnss_position(3))
        pose_theta = read_only_vars.gnss_position(3);
    end
end
if any(~isfinite(pose_xy)) && isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 2 ...
        && all(isfinite(public_vars.estimated_pose(1:2)))
    pose_xy = public_vars.estimated_pose(1:2);
end
if ~isfinite(pose_theta) && isfield(public_vars, 'estimated_pose') && numel(public_vars.estimated_pose) >= 3 ...
        && isfinite(public_vars.estimated_pose(3))
    if numel(public_vars.estimated_pose) >= 3 && isfinite(public_vars.estimated_pose(3))
        pose_theta = public_vars.estimated_pose(3);
    end
end
if any(~isfinite(pose_xy))
    return;
end

limits = read_only_vars.discrete_map.limits(:)';
xmin = limits(1);
ymin = limits(2);
xmax = limits(3);
ymax = limits(4);
edge_d = [pose_xy(1) - xmin, xmax - pose_xy(1), pose_xy(2) - ymin, ymax - pose_xy(2)];
min_edge = min(edge_d);
if ~isfinite(min_edge) || min_edge >= soft_margin
    return;
end

strength = max(0, min(1, (soft_margin - min_edge) / max(soft_margin - hard_margin, 1e-6)));
speed_scale = 1 - (1 - getfield_with_default(cfg, 'edge_v_scale_min', 0.22)) * strength;
speed_scale = max(getfield_with_default(cfg, 'edge_v_scale_min', 0.22), min(1.0, speed_scale));
active = min_edge < soft_margin;
if ~isfinite(pose_theta)
    heading_bias = 0;
    return;
end
center_xy = [(xmin + xmax) / 2, (ymin + ymax) / 2];
inward_heading = atan2(center_xy(2) - pose_xy(2), center_xy(1) - pose_xy(1));
heading_bias = wrap_to_pi_ws(inward_heading - pose_theta);
heading_bias = max(min(heading_bias, pi / 2), -pi / 2) * strength;
end

function tf = should_use_disambiguation_path(read_only_vars, public_vars)
tf = true;
if ~isfield(public_vars, 'disambiguate') || ~isfield(public_vars.disambiguate, 'path') ...
        || size(public_vars.disambiguate.path, 1) < 2
    return;
end
if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:3)))
    return;
end
if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    tf = false;
    return;
end

[front_min, ~, ~] = lidar_sector_minima(read_only_vars);
pose = public_vars.estimated_pose;
q = public_vars.localization_quality;
path_heading = atan2(public_vars.disambiguate.path(2,2) - pose(2), ...
    public_vars.disambiguate.path(2,1) - pose(1));
heading_error = abs(wrap_to_pi_ws(path_heading - pose(3)));

% Do not trust a planned disambiguation path unless localization is at
% least moderately coherent. Otherwise the robot can follow a path built
% from a wrong hypothesis into a wall.
scan_cost_ok = getfield_with_default(public_vars.localize, 'scan_cost_ok', 0.22);
single_commit_cluster = getfield_with_default(public_vars.localization_commit, 'pf_cluster_single_commit_m', 0.26);
cluster_ok = isfinite(getfield_with_default(q, 'pf_cluster_radius_m', nan)) ...
    && q.pf_cluster_radius_m <= max(0.45, 1.6 * single_commit_cluster);
mass_ok = isfinite(getfield_with_default(q, 'pf_dominant_mass', nan)) ...
    && q.pf_dominant_mass >= 0.84;
ratio_ok = isfinite(getfield_with_default(q, 'pf_top_ratio', nan)) ...
    && q.pf_top_ratio >= 1.10;
scan_ok = ~isfinite(getfield_with_default(q, 'scan_match_cost', nan)) ...
    || q.scan_match_cost <= max(0.14, 0.9 * scan_cost_ok);
hyp_ok = ~isfinite(getfield_with_default(q, 'pf_hypothesis_count', nan)) ...
    || q.pf_hypothesis_count <= 2;
if ~(cluster_ok && mass_ok && ratio_ok && scan_ok && hyp_ok)
    tf = false;
    return;
end

committed_heading = nan;
commit_active = false;
if isfield(public_vars, 'startup_motion')
    commit_active = isfield(public_vars.startup_motion, 'commit_counter') ...
        && public_vars.startup_motion.commit_counter > 0;
    if isfield(public_vars.startup_motion, 'commit_heading_rad')
        committed_heading = public_vars.startup_motion.commit_heading_rad;
    end
end

if commit_active && isfinite(committed_heading)
    reverse_from_commit = abs(wrap_to_pi_ws(path_heading - committed_heading)) > 105 * pi / 180;
    if reverse_from_commit && isfinite(front_min) && front_min > 1.35
        tf = false;
        return;
    end
end

if isfinite(front_min) && front_min > 1.45 && heading_error > 80 * pi / 180
    tf = false;
    return;
end
end

function bonus = informative_heading_bonus(read_only_vars, public_vars, heading, mode)
bonus = 0;
if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars, 'lidar_config')
    return;
end
[hypotheses, hyp_weights, hyp_meta] = top_localization_hypotheses(public_vars, 3);
if size(hypotheses, 1) < 2
    return;
end

step_len = 0.18;
if mode == "disambiguate"
    step_len = 0.24;
end
pred = cell(size(hypotheses, 1), 1);
for i = 1:size(hypotheses, 1)
    pose = hypotheses(i, :);
    pose(1:2) = pose(1:2) + step_len * [cos(heading), sin(heading)];
    pose(3) = wrap_to_pi_ws(heading);
    pred{i} = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
end

sep = 0;
stability_gain = 0;
for i = 1:numel(pred)
    stability_gain = stability_gain + hyp_weights(i) * getfield_with_default(hyp_meta(i), 'stability', 0);
    for j = (i + 1):numel(pred)
        pi = pred{i};
        pj = pred{j};
        valid = isfinite(pi) & isfinite(pj);
        if ~any(valid)
            continue;
        end
        dscan = mean(abs(pi(valid) - pj(valid)));
        sep = sep + hyp_weights(i) * hyp_weights(j) * dscan;
    end
end
bonus = min(0.55, 0.55 * sep + 0.12 * stability_gain);
end

function [hypotheses, hyp_weights, hyp_meta] = top_localization_hypotheses(public_vars, max_keep)
hypotheses = zeros(0, 3);
hyp_weights = zeros(0, 1);
hyp_meta = struct('mass', {}, 'radius_m', {}, 'scan_score', {}, 'age', {}, 'stability', {});
if ~isfield(public_vars, 'pf_stats') || ~isstruct(public_vars.pf_stats) ...
        || ~isfield(public_vars.pf_stats, 'top_hypotheses') || isempty(public_vars.pf_stats.top_hypotheses)
    return;
end

raw = public_vars.pf_stats.top_hypotheses;
keep = min(max_keep, numel(raw));
if keep <= 0
    return;
end
for i = 1:keep
    if ~isfield(raw(i), 'pose') || numel(raw(i).pose) < 3
        continue;
    end
    pose = raw(i).pose(1:3);
    if any(~isfinite(pose))
        continue;
    end
    hypotheses(end + 1, :) = pose; %#ok<AGROW>
    mass = getfield_with_default(raw(i), 'mass', nan);
    hyp_weights(end + 1, 1) = mass; %#ok<AGROW>
    hyp_meta(end + 1).mass = mass; %#ok<AGROW>
    hyp_meta(end).radius_m = getfield_with_default(raw(i), 'radius_m', nan);
    hyp_meta(end).scan_score = getfield_with_default(raw(i), 'scan_score', nan);
    hyp_meta(end).age = getfield_with_default(raw(i), 'age', 0);
    hyp_meta(end).stability = getfield_with_default(raw(i), 'stability', 0);
end
if isempty(hyp_weights)
    return;
end
hyp_weights = hyp_weights / max(sum(hyp_weights), 1e-12);
end

function motion_vector = apply_disambiguate_motion_limits(read_only_vars, public_vars, motion_vector)
if isempty(motion_vector) || numel(motion_vector) < 2
    motion_vector = [0, 0];
    return;
end
motion_vector = sanitize_motion_vector_ws(motion_vector);
L = read_only_vars.agent_drive.interwheel_dist;
[v_cmd, w_cmd] = motion_vector_to_vw_ws(motion_vector, L);
[front_min, ~, ~] = lidar_sector_minima(read_only_vars);

w_limit = 0.95;
if isfinite(front_min) && front_min < 0.85
    w_limit = 1.20;
end
w_cmd = max(min(w_cmd, w_limit), -w_limit);
if isfinite(front_min) && front_min > 1.00 && abs(w_cmd) > 0.75
    v_cmd = max(v_cmd, 0.08);
end
motion_vector = vw_to_motion_vector_ws(v_cmd, w_cmd, L);
end

function motion_vector = sanitize_motion_vector_ws(motion_vector)
if isempty(motion_vector) || numel(motion_vector) < 2 || any(~isfinite(motion_vector(1:2)))
    motion_vector = [0, 0];
else
    motion_vector = motion_vector(1:2);
end
end

function [v, w] = motion_vector_to_vw_ws(motion_vector, L)
vR = motion_vector(1);
vL = motion_vector(2);
v = 0.5 * (vR + vL);
w = (vR - vL) / max(L, 1e-6);
end

function motion_vector = vw_to_motion_vector_ws(v, w, L)
motion_vector = [v + 0.5 * L * w, v - 0.5 * L * w];
end

function metric = directional_open_metric(d, a, center_angle, half_width, clip_max)
mask = abs(wrap_to_pi_ws(a - center_angle)) <= half_width;
if ~any(mask)
    metric = inf;
    return;
end
vals = min(d(mask), clip_max);
metric = median(vals);
end

function [use_wall_follow, wall_side, align_counter, v, w] = wall_follow_command(read_only_vars, front, left, right, public_vars, prev_side, prev_align_counter, mode)
use_wall_follow = false;
wall_side = prev_side;
align_counter = prev_align_counter;
v = 0;
w = 0;

cfg = public_vars.wall_follow;
if ~isfinite(front) || front < cfg.front_clear_m
    wall_side = "none";
    align_counter = 0;
    return;
end

left_ok = isfinite(left) && left <= cfg.capture_dist_m;
right_ok = isfinite(right) && right <= cfg.capture_dist_m;

if wall_side == "left"
    if ~isfinite(left) || left > cfg.release_dist_m
        wall_side = "none";
        align_counter = 0;
    end
elseif wall_side == "right"
    if ~isfinite(right) || right > cfg.release_dist_m
        wall_side = "none";
        align_counter = 0;
    end
end

if wall_side == "none"
    if left_ok && right_ok
        if left <= right
            wall_side = "left";
        else
            wall_side = "right";
        end
    elseif left_ok
        wall_side = "left";
    elseif right_ok
        wall_side = "right";
    else
        align_counter = 0;
        return;
    end
end

parallel_err = estimate_wall_parallel_error(read_only_vars, wall_side);
if isfinite(parallel_err) && abs(parallel_err) <= cfg.align_tol_rad
    align_counter = align_counter + 1;
else
    align_counter = 0;
end

aligned = align_counter >= cfg.align_required_steps;
if isfinite(parallel_err) && (~aligned || abs(parallel_err) > cfg.align_release_rad)
    w = max(min(cfg.k_align * parallel_err, cfg.w_max), -cfg.w_max);
    v = cfg.v_align;
    use_wall_follow = true;
    return;
end

if wall_side == "left"
    err = cfg.desired_dist_m - left;
    w = -cfg.k_dist * err;
elseif wall_side == "right"
    err = cfg.desired_dist_m - right;
    w = cfg.k_dist * err;
else
    return;
end

w = max(min(w, cfg.w_max), -cfg.w_max);
if mode == "disambiguate"
    v = cfg.v_disambiguate;
else
    v = cfg.v_localize;
end
use_wall_follow = true;
end

function parallel_err = estimate_wall_parallel_error(read_only_vars, wall_side)
parallel_err = nan;
if ~isfield(read_only_vars, 'lidar_distances') || ~isfield(read_only_vars, 'lidar_config')
    return;
end

d = read_only_vars.lidar_distances(:);
a = wrap_to_pi_ws(read_only_vars.lidar_config(:));
valid = isfinite(d) & isfinite(a);
d = d(valid);
a = a(valid);
if isempty(d)
    return;
end

switch string(wall_side)
    case "left"
        mask = a >= 35 * pi / 180 & a <= 145 * pi / 180;
    case "right"
        mask = a <= -35 * pi / 180 & a >= -145 * pi / 180;
    otherwise
        return;
end

d = d(mask);
a = a(mask);
if numel(d) < 2
    return;
end

pts = [d .* cos(a), d .* sin(a)];
mu = mean(pts, 1);
X = pts - mu;
C = X' * X;
[V, D] = eig(C);
[~, idx] = max(diag(D));
dir = V(:, idx);
phi = atan2(dir(2), dir(1));
phi = wrap_to_pi_ws(phi);
if abs(phi) > pi / 2
    phi = phi - sign(phi) * pi;
end
parallel_err = phi;
end

function public_vars = update_track_progress(read_only_vars, public_vars)
public_vars.track.force_relocalize = false;
public_vars.track.force_replan = false;
public_vars.track.emergency_relocalize = false;
public_vars.track.emergency_replan = false;
public_vars.track.map_conflict_counter = max(0, getfield_with_default(public_vars.track, 'map_conflict_counter', 0)); %#ok<GFLD>
public_vars.track.commit_watchdog_counter = max(0, getfield_with_default(public_vars.track, 'commit_watchdog_counter', 0)); %#ok<GFLD>
public_vars.track.hard_path_dist_counter = max(0, getfield_with_default(public_vars.track, 'hard_path_dist_counter', 0)); %#ok<GFLD>
public_vars.track.offpath_replan_counter = max(0, getfield_with_default(public_vars.track, 'offpath_replan_counter', 0)); %#ok<GFLD>
public_vars.track.tight_front_replan_counter = max(0, getfield_with_default(public_vars.track, 'tight_front_replan_counter', 0)); %#ok<GFLD>
public_vars.track.replan_cooldown_counter = max(0, getfield_with_default(public_vars.track, 'replan_cooldown_counter', 0) - 1); %#ok<GFLD>
public_vars.debug.last_track_event = "none";
public_vars.debug.force_replan_reason = "none";
public_vars.debug.force_relocalize_reason = "none";
if ~isfield(public_vars, 'path') || isempty(public_vars.path) ...
        || ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:2)))
    return;
end

[closest_idx, path_dist] = nearest_path_index(public_vars.path, public_vars.estimated_pose(1:2));
goal_dist = norm(public_vars.estimated_pose(1:2) - read_only_vars.map.goal(1:2));
prev_goal_dist = public_vars.track.last_goal_distance;
goal_progress = isfinite(prev_goal_dist) ...
    && (prev_goal_dist - goal_dist) >= public_vars.track.goal_progress_reset_m;
best_goal_dist = getfield_with_default(public_vars.track, 'best_goal_distance', inf);
if ~isfinite(best_goal_dist)
    best_goal_dist = goal_dist;
end
if goal_dist <= best_goal_dist - public_vars.track.goal_progress_reset_m
    public_vars.track.best_goal_distance = goal_dist;
    best_goal_dist = goal_dist;
end
remaining_path_m = estimate_remaining_path_length(public_vars.path, closest_idx, public_vars.estimated_pose(1:2));
path_progress = isfinite(public_vars.track.last_remaining_path_m) ...
    && (public_vars.track.last_remaining_path_m - remaining_path_m) >= public_vars.track.path_progress_reset_m;
ambiguity_active = isfield(public_vars, 'environment_ambiguity') ...
    && isfield(public_vars.environment_ambiguity, 'active') ...
    && public_vars.environment_ambiguity.active;
q_track = getfield_with_default(public_vars, 'localization_quality', struct());
scan_quality = getfield_with_default(q_track, 'scan_match_cost', inf);
commit_state_track = string(getfield_with_default(getfield_with_default(public_vars, 'localization_commit', struct()), 'state', "search"));
weak_uniqueness_track = (~isfinite(getfield_with_default(q_track, 'pf_dominant_mass', nan)) ...
        || getfield_with_default(q_track, 'pf_dominant_mass', nan) < 0.84) ...
    || (~isfinite(getfield_with_default(q_track, 'pf_hypothesis_count', nan)) ...
        || getfield_with_default(q_track, 'pf_hypothesis_count', nan) > 1);
strong_track_localization = ~weak_uniqueness_track ...
    && isfinite(scan_quality) && scan_quality <= public_vars.localize.scan_cost_ok ...
    && isfinite(getfield_with_default(q_track, 'pf_cluster_radius_m', nan)) ...
    && getfield_with_default(q_track, 'pf_cluster_radius_m', nan) <= max(0.32, public_vars.localize.pf_cluster_ok) ...
    && isfinite(getfield_with_default(q_track, 'pf_dominant_mass', nan)) ...
    && getfield_with_default(q_track, 'pf_dominant_mass', nan) >= max(0.90, public_vars.localize.pf_dominant_mass_plan);
track_commit_like = commit_state_track == "committed" ...
    || (commit_state_track == "confirm" ...
        && strong_track_localization);
if track_commit_like
    public_vars.track.uncommitted_counter = 0;
else
    public_vars.track.uncommitted_counter = getfield_with_default(public_vars.track, 'uncommitted_counter', 0) + 1;
end

public_vars.track.last_path_index = closest_idx;
if closest_idx >= public_vars.track.best_path_index + public_vars.track.min_progress_index_jump ...
        || goal_progress || path_progress
    public_vars.track.best_path_index = closest_idx;
    public_vars.track.stall_steps = 0;
    public_vars.track.last_progress_step = public_vars.nav_state_step;
else
    public_vars.track.stall_steps = public_vars.track.stall_steps + 1;
end

away_from_goal = goal_dist > max(public_vars.track.away_goal_min_goal_dist_m, best_goal_dist + public_vars.track.away_goal_margin_m) ...
    && isfinite(path_dist) && path_dist <= public_vars.track.away_goal_path_dist_m ...
    && ~weak_uniqueness_track ...
    && isfinite(scan_quality) && scan_quality <= public_vars.localize.scan_cost_ok ...
    && ~ambiguity_active;
away_goal_progress_blocked = ~goal_progress && ~path_progress;
if away_from_goal && away_goal_progress_blocked ...
        && public_vars.track.stall_steps >= 4 ...
        && (goal_dist - prev_goal_dist) >= -public_vars.track.away_goal_step_m
    public_vars.track.away_goal_counter = getfield_with_default(public_vars.track, 'away_goal_counter', 0) + 1;
else
    public_vars.track.away_goal_counter = max(0, getfield_with_default(public_vars.track, 'away_goal_counter', 0) - 1);
end
hard_away_from_goal = goal_dist > max(public_vars.track.away_goal_min_goal_dist_m, best_goal_dist + public_vars.track.away_goal_hard_margin_m) ...
    && isfinite(path_dist) && path_dist <= max(0.45, 1.5 * public_vars.track.away_goal_path_dist_m) ...
    && away_goal_progress_blocked ...
    && public_vars.track.stall_steps >= 8 ...
    && ~weak_uniqueness_track ...
    && isfinite(scan_quality) && scan_quality <= public_vars.localize.scan_cost_ok ...
    && ~ambiguity_active;

near_goal_suspicious = false;
if goal_dist < public_vars.track.false_goal_radius_m
    ambiguity_near_goal = ambiguity_active ...
        && remaining_path_m > public_vars.track.near_goal_loop_remaining_path_m;
    off_path_conflict = path_dist > public_vars.track.commit_watchdog_path_dist_m ...
        && (weak_uniqueness_track || scan_quality > public_vars.track.commit_watchdog_scan_ema);
    near_goal_suspicious = public_vars.track.scan_mismatch_counter >= max(3, ceil(0.5 * public_vars.track.scan_mismatch_steps)) ...
        || ambiguity_near_goal ...
        || off_path_conflict;
    if near_goal_suspicious && goal_dist >= prev_goal_dist - 0.03
        public_vars.track.near_goal_loop_steps = public_vars.track.near_goal_loop_steps + 1;
    else
        public_vars.track.near_goal_loop_steps = 0;
    end
else
    public_vars.track.near_goal_loop_steps = 0;
end

fresh_commit_goal_collapse = isfield(public_vars.track, 'commit_counter') ...
    && public_vars.track.commit_counter > 0 ...
    && getfield_with_default(public_vars.track, 'entry_goal_distance', inf) > public_vars.track.fresh_commit_goal_jump_dist_m ...
    && goal_dist < public_vars.track.fresh_commit_false_goal_radius_m ...
    && isfinite(path_dist) && path_dist <= public_vars.track.fresh_commit_path_dist_m ...
    && (~isfinite(getfield_with_default(q_track, 'disagreement_xy_m', nan)) ...
        || weak_uniqueness_track ...
        || ambiguity_active);
if fresh_commit_goal_collapse
    public_vars.track.fresh_commit_false_goal_counter = getfield_with_default(public_vars.track, 'fresh_commit_false_goal_counter', 0) + 1;
else
    public_vars.track.fresh_commit_false_goal_counter = max(0, getfield_with_default(public_vars.track, 'fresh_commit_false_goal_counter', 0) - 1);
end
public_vars.track.last_goal_distance = goal_dist;
public_vars.track.last_remaining_path_m = remaining_path_m;

if public_vars.track.fresh_commit_false_goal_counter >= public_vars.track.fresh_commit_false_goal_steps
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "fresh_commit_false_goal";
end

near_goal_deadlock = goal_dist < public_vars.track.false_goal_caution_radius_m ...
    && public_vars.track.stall_steps >= public_vars.track.false_goal_deadlock_stall_steps ...
    && remaining_path_m <= max(0.45, 1.6 * public_vars.track.false_goal_radius_m) ...
    && ~goal_progress ...
    && ~path_progress;
if near_goal_deadlock
    if strong_track_localization && goal_dist > public_vars.track.false_goal_radius_m
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "false_goal_deadlock";
    else
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "false_goal_deadlock";
    end
end

false_goal_stop = goal_dist < public_vars.track.false_goal_radius_m ...
    && public_vars.track.stall_steps >= public_vars.track.false_goal_stop_stall_steps ...
    && remaining_path_m <= max(0.35, 1.4 * public_vars.track.false_goal_radius_m) ...
    && (near_goal_suspicious || weak_uniqueness_track);
if false_goal_stop
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "false_goal_stall";
end

false_goal_offpath = goal_dist < public_vars.track.false_goal_caution_radius_m ...
    && public_vars.track.stall_steps >= public_vars.track.false_goal_relocalize_stall_steps ...
    && isfinite(path_dist) && path_dist > max(0.80, 0.60 * public_vars.track.hard_path_dist_m) ...
    && remaining_path_m > public_vars.track.near_goal_loop_remaining_path_m ...
    && (near_goal_suspicious || weak_uniqueness_track || scan_quality > public_vars.track.commit_watchdog_scan_ema);
if false_goal_offpath
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "false_goal_offpath";
end

confirm_false_goal = goal_dist < getfield_with_default(public_vars.track, 'false_goal_confirm_goal_radius_m', 0.9) ...
    && public_vars.track.stall_steps >= getfield_with_default(public_vars.track, 'false_goal_confirm_stall_steps', 6) ...
    && ~track_commit_like ...
    && isfinite(path_dist) && path_dist <= getfield_with_default(public_vars.track, 'false_goal_confirm_path_dist_m', 0.20);
if confirm_false_goal
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "false_goal_uncommitted";
end

track_uncommitted = getfield_with_default(public_vars.track, 'uncommitted_counter', 0) >= ...
        getfield_with_default(public_vars.track, 'uncommitted_relocalize_steps', 24) ...
    && public_vars.track.stall_steps >= 4 ...
    && isfinite(path_dist) && path_dist <= max(0.22, 1.2 * public_vars.track.away_goal_path_dist_m);
if track_uncommitted
    if strong_track_localization
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "track_uncommitted";
    else
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "track_uncommitted";
    end
end

if public_vars.track.stall_steps > public_vars.track.max_stall_steps && goal_dist > 1.2 ...
        && (path_dist > 0.35 || ambiguity_active)
    if goal_dist < public_vars.track.false_goal_caution_radius_m ...
            && isfinite(path_dist) && path_dist > 0.55
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "track_stall_false_goal";
    else
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "track_stall";
    end
end

if isfinite(path_dist) && path_dist > public_vars.track.hard_path_dist_m
    public_vars.track.hard_path_dist_counter = public_vars.track.hard_path_dist_counter + 1;
else
    public_vars.track.hard_path_dist_counter = max(0, public_vars.track.hard_path_dist_counter - 1);
end
if public_vars.track.hard_path_dist_counter >= public_vars.track.hard_path_dist_steps
    if strong_track_localization && goal_dist > 1.0
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "hard_path_distance";
    else
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "hard_path_distance";
    end
end

limited_goal_progress = ~goal_progress && isfinite(prev_goal_dist) ...
    && (goal_dist - prev_goal_dist) >= -getfield_with_default(public_vars.track, 'offpath_replan_goal_progress_m', 0.015);
offpath_replan_ready = strong_track_localization ...
    && goal_dist > 1.2 ...
    && isfinite(path_dist) ...
    && path_dist >= getfield_with_default(public_vars.track, 'offpath_replan_dist_m', 0.55) ...
    && limited_goal_progress ...
    && ~path_progress ...
    && getfield_with_default(public_vars.track, 'replan_cooldown_counter', 0) <= 0;
if offpath_replan_ready
    public_vars.track.offpath_replan_counter = public_vars.track.offpath_replan_counter + 1;
    if path_dist >= getfield_with_default(public_vars.track, 'offpath_replan_hard_dist_m', 0.80)
        public_vars.track.offpath_replan_counter = public_vars.track.offpath_replan_counter + 1;
    end
else
    public_vars.track.offpath_replan_counter = max(0, public_vars.track.offpath_replan_counter - 1);
end
if public_vars.track.offpath_replan_counter >= getfield_with_default(public_vars.track, 'offpath_replan_steps', 14)
    public_vars.track.force_replan = true;
    public_vars.debug.force_replan_reason = "offpath_progress";
    public_vars.track.replan_cooldown_counter = getfield_with_default(public_vars.track, 'replan_cooldown_steps', 45);
end

if near_goal_suspicious && public_vars.track.near_goal_loop_steps > public_vars.track.max_near_goal_loop_steps
    public_vars.track.force_relocalize = true;
    public_vars.debug.force_relocalize_reason = "near_goal_loop";
end

scan_cost = inf;
if isfield(read_only_vars, 'lidar_distances') && ~isempty(read_only_vars.lidar_distances) ...
        && any(isfinite(read_only_vars.lidar_distances)) ...
        && isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose) ...
        && all(isfinite(public_vars.estimated_pose(1:3)))
    pred = compute_lidar_measurement(read_only_vars.map, public_vars.estimated_pose, read_only_vars.lidar_config);
    scan_cost = lidar_match_cost(pred, read_only_vars.lidar_distances);
end
public_vars.track.scan_mismatch_cost = scan_cost;
if isfinite(scan_cost) && scan_cost > public_vars.track.scan_mismatch_trigger
    public_vars.track.scan_mismatch_counter = public_vars.track.scan_mismatch_counter + 1;
elseif isfinite(scan_cost) && scan_cost <= public_vars.track.scan_mismatch_warn
    public_vars.track.scan_mismatch_counter = 0;
end

commit_active = isfield(public_vars, 'localization_commit') ...
    && isfield(public_vars.localization_commit, 'state') ...
    && string(public_vars.localization_commit.state) == "committed";
scan_ema = getfield_with_default(public_vars.localization_quality, 'scan_match_ema', scan_cost);
path_trend = getfield_with_default(public_vars.localization_quality, 'path_distance_trend', 0);
[front_watchdog, left_watchdog, right_watchdog] = lidar_sector_minima(read_only_vars);
front_watchdog_open = (~isfinite(front_watchdog)) || front_watchdog >= 1.20;
left_watchdog_open = (~isfinite(left_watchdog)) || left_watchdog >= 0.90;
right_watchdog_open = (~isfinite(right_watchdog)) || right_watchdog >= 0.90;
open_track_area = front_watchdog_open && left_watchdog_open && right_watchdog_open;
stable_commit_localization = isfinite(path_dist) && path_dist <= 0.30 ...
    && isfinite(getfield_with_default(q_track, 'disagreement_xy_m', nan)) ...
    && getfield_with_default(q_track, 'disagreement_xy_m', nan) <= 0.35 ...
    && ~weak_uniqueness_track;
protected_track_commit = isfinite(path_dist) && path_dist <= 0.18 ...
    && isfinite(getfield_with_default(q_track, 'disagreement_xy_m', nan)) ...
    && getfield_with_default(q_track, 'disagreement_xy_m', nan) <= 0.45 ...
    && ~weak_uniqueness_track;
outdoor_gnss_commit = open_track_area ...
    && getfield_with_default(q_track, 'gnss_valid', false) ...
    && isfinite(path_dist) && path_dist <= 0.60 ...
    && isfinite(getfield_with_default(q_track, 'disagreement_xy_m', nan)) ...
    && getfield_with_default(q_track, 'disagreement_xy_m', nan) <= 0.60;
commit_watchdog_bad = commit_active ...
    && goal_dist > public_vars.track.commit_watchdog_goal_dist_m ...
    && ~protected_track_commit ...
    && ~outdoor_gnss_commit ...
    && ~(open_track_area && stable_commit_localization ...
         && isfinite(scan_cost) && scan_cost <= 0.35 ...
         && isfinite(scan_ema) && scan_ema <= 1.10) ...
    && ((isfinite(scan_ema) && scan_ema > public_vars.track.commit_watchdog_scan_ema ...
         && path_dist > public_vars.track.commit_watchdog_path_dist_m) ...
        || (isfinite(scan_cost) && scan_cost > public_vars.track.commit_watchdog_scan_cost ...
         && path_dist > public_vars.track.commit_watchdog_path_dist_m) ...
        || (isfinite(path_trend) && path_trend > public_vars.track.commit_watchdog_path_trend_m) ...
        || public_vars.track.scan_mismatch_counter >= max(3, ceil(0.5 * public_vars.track.scan_mismatch_steps)) ...
        || public_vars.localize.contradiction_counter >= max(3, ceil(0.5 * public_vars.localize.contradiction_steps)));
if commit_watchdog_bad
    public_vars.track.commit_watchdog_counter = public_vars.track.commit_watchdog_counter + 1;
else
    public_vars.track.commit_watchdog_counter = max(0, public_vars.track.commit_watchdog_counter - 1);
end

if public_vars.track.commit_watchdog_counter >= public_vars.track.commit_watchdog_steps
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "commit_watchdog";
end

if path_dist > public_vars.track.max_path_distance_m ...
        && public_vars.track.scan_mismatch_counter >= max(4, ceil(0.5 * public_vars.track.scan_mismatch_steps)) ...
        && public_vars.track.stall_steps >= 10
    public_vars.track.force_replan = true;
    public_vars.debug.force_replan_reason = "path_distance_mismatch_stall";
end

front_min = front_watchdog;
left_min = left_watchdog;
right_min = right_watchdog;
prev_limiter = "none";
if isfield(public_vars, 'motion_debug') && isfield(public_vars.motion_debug, 'limiter') ...
        && ~isempty(public_vars.motion_debug.limiter)
    prev_limiter = string(public_vars.motion_debug.limiter);
end
if isfinite(front_min) && front_min < public_vars.track.near_collision_front_m && goal_dist > 1.0
    public_vars.track.front_risk_counter = public_vars.track.front_risk_counter + 1;
else
    public_vars.track.front_risk_counter = 0;
end
side_min = min(left_min, right_min);
if isfinite(side_min) && side_min < public_vars.track.near_collision_side_m && goal_dist > 1.0
    public_vars.track.side_risk_counter = public_vars.track.side_risk_counter + 1;
else
    public_vars.track.side_risk_counter = 0;
end

good_track_localization = false;
if isfield(public_vars, 'localization_quality')
    q = public_vars.localization_quality;
    good_track_localization = isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.25 ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90 ...
        && isfinite(scan_cost) && scan_cost <= public_vars.localize.scan_cost_ok;
end

map_conflict = prev_limiter == "map" ...
    && isfinite(front_min) && front_min > 0.60 ...
    && isfinite(path_dist) && path_dist > 0.20 ...
    && public_vars.track.stall_steps >= 12 ...
    && ~good_track_localization;
if map_conflict
    public_vars.track.map_conflict_counter = public_vars.track.map_conflict_counter + 1;
else
    public_vars.track.map_conflict_counter = 0;
end

if public_vars.track.front_risk_counter >= public_vars.track.front_risk_steps_relocalize ...
        && goal_dist > 1.0 && public_vars.track.stall_steps >= 14
    if good_track_localization
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "front_collision_risk";
        if isfinite(front_min) && front_min < 0.18
            public_vars.track.emergency_replan = true;
        end
    else
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "front_collision_risk";
    end
end
if public_vars.track.side_risk_counter >= public_vars.track.side_risk_steps_relocalize ...
        && goal_dist > 1.0 && public_vars.track.stall_steps >= 12
    if good_track_localization
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "side_collision_risk";
        if isfinite(side_min) && side_min < 0.12
            public_vars.track.emergency_replan = true;
        end
    else
        public_vars.track.force_relocalize = true;
        public_vars.track.emergency_relocalize = true;
        public_vars.debug.force_relocalize_reason = "side_collision_risk";
    end
end
if public_vars.track.map_conflict_counter >= public_vars.track.map_conflict_steps_relocalize ...
        && goal_dist > 1.0
    public_vars.track.force_relocalize = true;
    public_vars.track.emergency_relocalize = true;
    public_vars.debug.force_relocalize_reason = "map_scan_conflict";
end

tight_front_replan_ready = strong_track_localization ...
    && goal_dist > 1.0 ...
    && isfinite(front_min) && front_min <= getfield_with_default(public_vars.track, 'tight_front_replan_front_m', 0.42) ...
    && isfinite(path_dist) && path_dist >= getfield_with_default(public_vars.track, 'tight_front_replan_path_dist_m', 0.18) ...
    && getfield_with_default(public_vars.track, 'replan_cooldown_counter', 0) <= 0;
if tight_front_replan_ready
    public_vars.track.tight_front_replan_counter = public_vars.track.tight_front_replan_counter + 1;
else
    public_vars.track.tight_front_replan_counter = max(0, public_vars.track.tight_front_replan_counter - 1);
end
if public_vars.track.tight_front_replan_counter >= getfield_with_default(public_vars.track, 'tight_front_replan_steps', 6)
    public_vars.track.force_replan = true;
    public_vars.debug.force_replan_reason = "tight_front_clearance";
    public_vars.track.replan_cooldown_counter = getfield_with_default(public_vars.track, 'replan_cooldown_steps', 45);
    if isfinite(front_min) && front_min < 0.30
        public_vars.track.emergency_replan = true;
    end
end

good_pf_for_replan = false;
if isfield(public_vars, 'localization_quality')
    q = public_vars.localization_quality;
    good_pf_for_replan = isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.20 ...
        && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= 0.90;
end
replan_scan_confirmed = isfinite(public_vars.track.scan_mismatch_cost) ...
    && public_vars.track.scan_mismatch_counter >= max(3, ceil(0.5 * public_vars.track.scan_mismatch_steps));
if good_pf_for_replan && goal_dist > 1.0
    if public_vars.track.front_risk_counter >= public_vars.track.front_risk_steps_replan ...
            && public_vars.track.stall_steps >= 12 ...
            && replan_scan_confirmed ...
            && isfinite(front_min) && front_min < public_vars.track.near_collision_replan_front_m
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "front_replan_risk";
        if front_min < 0.20 && public_vars.track.stall_steps >= 10
            public_vars.track.emergency_replan = true;
        end
    end
    if public_vars.track.side_risk_counter >= public_vars.track.side_risk_steps_replan ...
            && public_vars.track.stall_steps >= 12 ...
            && replan_scan_confirmed ...
            && isfinite(side_min) && side_min < public_vars.track.near_collision_replan_side_m
        public_vars.track.force_replan = true;
        public_vars.debug.force_replan_reason = "side_replan_risk";
        if side_min < 0.08 && public_vars.track.stall_steps >= 10
            public_vars.track.emergency_replan = true;
        end
    end
end

if goal_dist < public_vars.track.false_goal_caution_radius_m
    ambiguity_uncleared = isfield(public_vars, 'environment_ambiguity') ...
        && isfield(public_vars.environment_ambiguity, 'clear_counter') ...
        && public_vars.environment_ambiguity.clear_counter < public_vars.ambiguity.clear_required_steps;
    weak_uniqueness = false;
    if isfield(public_vars, 'localization_quality')
        q = public_vars.localization_quality;
        weak_uniqueness = (~isfinite(q.pf_dominant_mass) || q.pf_dominant_mass < 0.82) ...
            || (~isfinite(q.pf_top_ratio) || q.pf_top_ratio < 1.08);
    end
    scan_mismatch_bad = isfield(public_vars.track, 'scan_mismatch_counter') ...
        && public_vars.track.scan_mismatch_counter >= public_vars.track.scan_mismatch_steps;
    if ambiguity_uncleared && scan_mismatch_bad ...
            && public_vars.track.stall_steps >= public_vars.track.false_goal_relocalize_stall_steps
        public_vars.track.force_relocalize = true;
        public_vars.debug.force_relocalize_reason = "false_goal_ambiguity";
    elseif weak_uniqueness && scan_mismatch_bad ...
            && public_vars.track.stall_steps >= public_vars.track.false_goal_relocalize_stall_steps
        public_vars.track.force_relocalize = true;
        public_vars.debug.force_relocalize_reason = "false_goal_weak_uniqueness";
    end
end

away_goal_replan_ready = public_vars.track.away_goal_counter >= max( ...
        public_vars.track.away_goal_trigger_steps, ...
        getfield_with_default(public_vars.track, 'away_goal_min_counter_replan', 3)) ...
    && public_vars.track.stall_steps >= 6 ...
    && isfinite(path_dist) ...
    && path_dist >= getfield_with_default(public_vars.track, 'away_goal_min_path_dist_replan_m', 0.22) ...
    && getfield_with_default(public_vars.track, 'replan_cooldown_counter', 0) <= 0;
hard_away_goal_replan_ready = hard_away_from_goal ...
    && getfield_with_default(public_vars.track, 'replan_cooldown_counter', 0) <= 0;
if away_goal_replan_ready || hard_away_goal_replan_ready
    public_vars.track.force_replan = true;
    public_vars.debug.force_replan_reason = "away_from_goal";
    public_vars.track.replan_cooldown_counter = getfield_with_default(public_vars.track, 'replan_cooldown_steps', 45);
    if public_vars.track.stall_steps >= 10
        public_vars.track.emergency_replan = true;
    end
end

if public_vars.track.force_replan
    public_vars.debug.last_track_event = "force_replan";
elseif public_vars.track.force_relocalize
    public_vars.debug.last_track_event = "force_relocalize";
end

if isfield(public_vars, 'localization_quality')
    public_vars.localization_quality.path_distance_m = path_dist;
    public_vars.localization_quality.goal_distance_m = goal_dist;
    public_vars.localization_quality.best_path_index = public_vars.track.best_path_index;
    public_vars.localization_quality.track_stall_steps = public_vars.track.stall_steps;
    public_vars.localization_quality.near_goal_loop_steps = public_vars.track.near_goal_loop_steps;
    public_vars.localization_quality.track_scan_mismatch_cost = scan_cost;
    public_vars.localization_quality.track_scan_mismatch_counter = public_vars.track.scan_mismatch_counter;
    public_vars.localization_quality.track_map_conflict_counter = public_vars.track.map_conflict_counter;
    public_vars.localization_quality.commit_watchdog_counter = public_vars.track.commit_watchdog_counter;
    public_vars.localization_quality.hard_path_dist_counter = public_vars.track.hard_path_dist_counter;
    public_vars.localization_quality.reseed_attempt_count = getfield_with_default(public_vars.localize, 'reseed_attempt_count', 0);
    public_vars.localization_quality.reseed_success_count = getfield_with_default(public_vars.localize, 'reseed_success_count', 0);
    public_vars.localization_quality.reseed_last_score = getfield_with_default(public_vars.localize, 'last_reseed_score', 0);
    public_vars.localization_quality.post_reseed_hold_counter = getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0);
    public_vars.localization_quality.disambiguation_goal_switches = getfield_with_default(public_vars.disambiguate, 'goal_switch_count', 0);
end
end

function value = getfield_with_default(s, field_name, fallback)
if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
    value = s.(field_name);
else
    value = fallback;
end
end

function tf = is_hard_false_goal_reset_reason(reason)
reason = string(reason);
tf = any(reason == ["near_goal_loop", "false_goal_deadlock", "false_goal_stall", ...
    "false_goal_offpath", "false_goal_uncommitted", "track_uncommitted", "fresh_commit_false_goal", "track_stall_false_goal", ...
    "false_goal_ambiguity", "false_goal_weak_uniqueness"]);
end

function [idx, dist] = nearest_path_index(path, xy)
if isempty(path)
    idx = 1;
    dist = inf;
    return;
end
d = vecnorm(path(:, 1:2) - xy(:)', 2, 2);
[dist, idx] = min(d);
end

function remaining_m = estimate_remaining_path_length(path, idx, xy)
remaining_m = inf;
if isempty(path) || idx < 1 || idx > size(path, 1) || any(~isfinite(xy))
    return;
end

remaining_m = norm(path(idx, 1:2) - xy(:)');
if idx < size(path, 1)
    remaining_m = remaining_m + sum(vecnorm(diff(path(idx:end, 1:2), 1, 1), 2, 2));
end
end

function tf = should_trigger_global_reseed(public_vars, lidar_valid)
tf = false;
if ~lidar_valid || ~isfield(public_vars, 'localize') || ~public_vars.localize.allow_global_reseed
    return;
end
score = localization_crisis_score(public_vars);
steps_since = public_vars.nav_state_step - getfield_with_default(public_vars.localize, 'last_reseed_step', -inf);
period_min = getfield_with_default(public_vars.localize, 'reseed_period_min', 18);
period_max = getfield_with_default(public_vars.localize, 'reseed_period_max', 90);
adaptive_period = round(period_max - (period_max - period_min) * min(max(score, 0), 1));
adaptive_period = max(period_min, min(period_max, adaptive_period));
fast_recovery = getfield_with_default(public_vars.localize, 'fast_recovery', false);
if fast_recovery
    fast_reseed_period = max(20, round(0.70 * adaptive_period));
    sufficient_motion = getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0) >= 0.6 ...
        || getfield_with_default(public_vars.localize, 'cmd_turn_accum', 0) >= 0.9;
    if public_vars.nav_state_step >= fast_reseed_period && steps_since >= fast_reseed_period && sufficient_motion
        tf = true;
        return;
    end
end
tf = score >= getfield_with_default(public_vars.localize, 'reseed_crisis_trigger', 0.52) ...
    && steps_since >= adaptive_period;
end

function score = localization_crisis_score(public_vars)
score = 0;
q = getfield_with_default(public_vars, 'localization_quality', struct());
if isfield(q, 'scan_match_ema') && isfinite(q.scan_match_ema)
    score = score + 0.35 * min(q.scan_match_ema / 0.25, 1.5);
elseif isfield(q, 'scan_match_cost') && isfinite(q.scan_match_cost)
    score = score + 0.35 * min(q.scan_match_cost / 0.25, 1.5);
end
if isfield(q, 'pf_cluster_radius_m') && isfinite(q.pf_cluster_radius_m)
    score = score + 0.20 * min(q.pf_cluster_radius_m / 0.70, 1.5);
end
if isfield(q, 'disagreement_ema') && isfinite(q.disagreement_ema)
    score = score + 0.20 * min(q.disagreement_ema / 0.90, 1.5);
elseif isfield(q, 'disagreement_xy_m') && isfinite(q.disagreement_xy_m)
    score = score + 0.20 * min(q.disagreement_xy_m / 0.90, 1.5);
end
if isfield(q, 'pf_hypothesis_count') && isfinite(q.pf_hypothesis_count)
    score = score + 0.10 * min(max(q.pf_hypothesis_count - 1, 0) / 3, 1.0);
end
score = score + 0.10 * min(getfield_with_default(public_vars.localize, 'contradiction_counter', 0) / ...
    max(getfield_with_default(public_vars.localize, 'contradiction_steps', 8), 1), 1.0);
score = min(max(score, 0), 1.5);
end

function public_vars = global_lidar_reseed(read_only_vars, public_vars)
if ~isfield(read_only_vars, 'discrete_map') || ~isfield(read_only_vars.discrete_map, 'map')
    return;
end
if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances) ...
        || all(~isfinite(read_only_vars.lidar_distances))
    return;
end

grid = read_only_vars.discrete_map.map;
[ny, nx] = size(grid);
limits = read_only_vars.discrete_map.limits;
crisis_score = localization_crisis_score(public_vars);
stride = max(1, round(public_vars.localize.reseed_stride + 2 * max(0, 1 - min(crisis_score, 1))));
ori_n = max(4, round(public_vars.localize.reseed_orientations * (0.60 + 0.55 * min(crisis_score, 1))));
ori = linspace(-pi, pi, ori_n + 1);
ori(end) = [];
public_vars.localize.reseed_attempt_count = getfield_with_default(public_vars.localize, 'reseed_attempt_count', 0) + 1;
public_vars.localize.last_reseed_step = public_vars.nav_state_step;
public_vars.localize.last_reseed_score = crisis_score;

candidate_list = zeros(0, 4);

for iy = 1:stride:ny
    for ix = 1:stride:nx
        if grid(iy, ix) ~= 0
            continue;
        end
        xy = discrete_to_world_ws(ix, iy, limits, [nx ny]);
        for k = 1:numel(ori)
            pose = [xy, ori(k)];
            pred = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
            c = lidar_match_cost(pred, read_only_vars.lidar_distances);
            candidate_list(end + 1, :) = [pose, c]; %#ok<AGROW>
        end
    end
end

if isempty(candidate_list)
    return;
end

[~, order] = sort(candidate_list(:, 4), 'ascend');
candidate_list = candidate_list(order, :);
max_keep = 16;
min_dist = 0.75;
if crisis_score >= 0.75
    max_keep = 24;
    min_dist = 0.60;
end
selected = select_diverse_candidates(candidate_list, max_keep, min_dist);
if isempty(selected)
    return;
end

best_cost = selected(1, 4);
if size(selected, 1) >= 2
    second_cost = selected(2, 4);
else
    second_cost = inf;
end
quality_gap = second_cost - best_cost;

reseed_fraction = getfield_with_default(public_vars.localize, 'reseed_fraction_min', 0.40) ...
    + min(max(max(quality_gap, crisis_score), 0), 1) ...
    * (getfield_with_default(public_vars.localize, 'reseed_fraction_max', 0.85) ...
       - getfield_with_default(public_vars.localize, 'reseed_fraction_min', 0.40));

N = size(public_vars.particles, 1);
replace_n = min(N, max(40, round(reseed_fraction * N)));
idx_replace = randperm(N, replace_n);
num_hyp = size(selected, 1);

costs = selected(:, 4);
cost_span = max(costs) - min(costs);
temp = max(0.04, 0.18 * max(cost_span, 0.15));
hyp_w = exp(-(costs - min(costs)) / temp);
hyp_w = hyp_w / max(sum(hyp_w), 1e-12);
if quality_gap < 0.20 || crisis_score >= 0.75
    hyp_w = hyp_w .^ 0.60;
    hyp_w = hyp_w + 0.02;
    hyp_w = hyp_w / max(sum(hyp_w), 1e-12);
end

counts = max(8, round(replace_n * hyp_w));
while sum(counts) > replace_n
    [~, j] = max(counts);
    counts(j) = counts(j) - 1;
end
while sum(counts) < replace_n
    [~, j] = max(hyp_w);
    counts(j) = counts(j) + 1;
end

cursor = 1;
new_weights = zeros(N, 1);
keep_mask = true(N, 1);
keep_mask(idx_replace) = false;
if isfield(public_vars, 'particle_weights') && numel(public_vars.particle_weights) == N
    kept_weights = public_vars.particle_weights(:);
    kept_weights(~keep_mask) = 0;
    keep_fraction = 0.35;
    if quality_gap < 0.20 || crisis_score >= 0.75
        keep_fraction = 0.20;
    end
    kept_weights = keep_fraction * kept_weights / max(sum(kept_weights), 1e-12);
    new_weights = new_weights + kept_weights;
end

xy_sigma = 0.08;
th_sigma = 0.14;
if quality_gap < 0.15
    xy_sigma = 0.14;
    th_sigma = 0.20;
end
if quality_gap < 0.08 || crisis_score >= 0.85
    xy_sigma = max(xy_sigma, 0.18);
    th_sigma = max(th_sigma, 0.28);
end

for h = 1:num_hyp
    n_h = counts(h);
    if n_h <= 0
        continue;
    end
    base_pose = selected(h, 1:3);
    for j = 1:n_h
        idx = idx_replace(cursor);
        cursor = cursor + 1;
        public_vars.particles(idx, 1:2) = base_pose(1:2) + xy_sigma * randn(1, 2);
        public_vars.particles(idx, 3) = wrap_to_pi_ws(base_pose(3) + th_sigma * randn());
        inject_fraction_w = 0.65;
        if quality_gap < 0.20 || crisis_score >= 0.75
            inject_fraction_w = 0.80;
        end
        new_weights(idx) = inject_fraction_w * hyp_w(h) / n_h;
    end
end

if sum(new_weights) <= 0 || any(~isfinite(new_weights))
    public_vars.particle_weights = ones(N, 1) / N;
else
    public_vars.particle_weights = new_weights / sum(new_weights);
end
public_vars.localize.reseed_success_count = getfield_with_default(public_vars.localize, 'reseed_success_count', 0) + 1;
public_vars.localize.stable_counter = 0;
public_vars.localize.post_reseed_hold_counter = max(getfield_with_default(public_vars.localize, 'post_reseed_hold_counter', 0), ...
    getfield_with_default(public_vars.localize, 'post_reseed_hold_steps', 0));
public_vars.localize.info_gain_recovery_counter = max(getfield_with_default(public_vars.localize, 'info_gain_recovery_counter', 0), ...
    getfield_with_default(public_vars.localize, 'info_gain_recovery_steps', 0));
end

function selected = select_diverse_candidates(candidate_list, max_keep, min_dist)
selected = zeros(0, 4);
for i = 1:size(candidate_list, 1)
    cand = candidate_list(i, :);
    if isempty(selected)
        selected = cand;
    else
        d = vecnorm(selected(:, 1:2) - cand(1:2), 2, 2);
        if all(d >= min_dist)
            selected(end + 1, :) = cand; %#ok<AGROW>
        end
    end
    if size(selected, 1) >= max_keep
        break;
    end
end
if isempty(selected)
    selected = candidate_list(1, :);
end
end

function xy = discrete_to_world_ws(ix, iy, limits, dims)
nx = dims(1);
ny = dims(2);
if nx <= 1
    x = limits(1);
else
    x = limits(1) + (ix - 1) * (limits(3) - limits(1)) / (nx - 1);
end
if ny <= 1
    y = limits(2);
else
    y = limits(2) + (iy - 1) * (limits(4) - limits(2)) / (ny - 1);
end
xy = [x, y];
end

function public_vars = prepare_disambiguation_path(read_only_vars, public_vars)
public_vars.disambiguate.path = [];
public_vars.disambiguate.goal_xy = [nan, nan];
public_vars.disambiguate.goal_sector_id = nan;

[goal_xy, found] = find_informative_goal(read_only_vars, public_vars);
if ~found
    return;
end

tmp_read_only = read_only_vars;
tmp_read_only.map.goal = goal_xy(:)';
tmp_public = public_vars;
tmp_public.path = [];
tmp_public.replan_path = true;

path = plan_path(tmp_read_only, tmp_public);
if isempty(path) || size(path, 1) < 2
    return;
end

public_vars.disambiguate.path = path;
public_vars.disambiguate.goal_xy = goal_xy(:)';
public_vars.disambiguate.goal_sector_id = heading_sector_id_ws( ...
    atan2(goal_xy(2) - public_vars.estimated_pose(2), goal_xy(1) - public_vars.estimated_pose(1)), ...
    public_vars.disambiguate.sector_width_rad);
public_vars.disambiguate.goal_switch_count = getfield_with_default(public_vars.disambiguate, 'goal_switch_count', 0) + 1;
public_vars.skip_start_alignment = false;
end

function tf = startup_localization_informative(public_vars, lidar_valid)
tf = true;
if nargin < 2
    lidar_valid = true;
end
if ~lidar_valid
    return;
end

if ~isfield(public_vars, 'localization_quality') || isempty(public_vars.localization_quality)
    tf = false;
    return;
end
q = public_vars.localization_quality;
scan_change = inf;
if isfield(public_vars, 'environment_ambiguity') && isfield(public_vars.environment_ambiguity, 'scan_change') ...
        && isfinite(public_vars.environment_ambiguity.scan_change)
    scan_change = public_vars.environment_ambiguity.scan_change;
end
clear_steps = get_ambiguity_clear_counter_ws(public_vars);
cmd_dist = getfield_with_default(public_vars.localize, 'cmd_distance_accum', 0);
cmd_turn = getfield_with_default(public_vars.localize, 'cmd_turn_accum', 0);
has_motion = cmd_dist >= public_vars.localize.min_informative_cmd_distance ...
    && cmd_turn >= public_vars.localize.min_informative_cmd_turn;
has_uniqueness = isfinite(q.pf_top_ratio) && q.pf_top_ratio >= public_vars.localize.min_informative_pf_ratio ...
    && isfinite(q.pf_dominant_mass) && q.pf_dominant_mass >= public_vars.localize.min_informative_pf_mass;
has_scan_change = isfinite(scan_change) && scan_change >= public_vars.localize.min_informative_scan_change;
has_partial_motion = cmd_dist >= 0.65 * public_vars.localize.min_informative_cmd_distance ...
    && cmd_turn >= 0.45 * public_vars.localize.min_informative_cmd_turn;
strong_uniqueness = has_uniqueness ...
    && isfinite(q.pf_cluster_radius_m) && q.pf_cluster_radius_m <= 0.24;
tf = (has_motion && clear_steps >= public_vars.ambiguity.clear_required_steps && (has_scan_change || has_uniqueness)) ...
    || (strong_uniqueness && has_partial_motion && clear_steps >= 2);
end

function [goal_xy, found] = find_informative_goal(read_only_vars, public_vars)
goal_xy = [nan, nan];
found = false;

if ~isfield(read_only_vars, 'discrete_map') || ~isfield(read_only_vars.discrete_map, 'map')
    return;
end
if ~isfield(public_vars, 'estimated_pose') || isempty(public_vars.estimated_pose) ...
        || any(~isfinite(public_vars.estimated_pose(1:2)))
    return;
end

dmap = read_only_vars.discrete_map;
grid = dmap.map;
limits = dmap.limits;
dims = dmap.dims;
verify_selecting = isfield(public_vars, 'verify') ...
    && isfield(public_vars.verify, 'selecting') ...
    && public_vars.verify.selecting;
final_goal_xy = [nan, nan];
current_goal_dist = nan;
if verify_selecting && isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'goal') ...
        && numel(read_only_vars.map.goal) >= 2 && all(isfinite(read_only_vars.map.goal(1:2)))
    final_goal_xy = read_only_vars.map.goal(1:2);
    current_goal_dist = norm(public_vars.estimated_pose(1:2) - final_goal_xy);
end
[ny, nx] = size(grid);
if nx ~= dims(1) || ny ~= dims(2)
    dims = [nx, ny];
end

planner_clearance = getfield_with_default(public_vars, 'planner_clearance_m', getfield_with_default(public_vars, 'path_clearance_m', 0.25));
inflated = inflate_grid_ws(grid, limits, dims, max(0.2, planner_clearance));
[sx, sy] = continous_to_discrete_coords_ws(public_vars.estimated_pose(1), public_vars.estimated_pose(2), limits, dims);
[sx, sy] = clip_to_grid_ws(sx, sy, nx, ny);
[sx, sy] = snap_to_nearest_free_ws(inflated, sx, sy);
[clear_map, cell_m] = clearance_map_ws(inflated, limits, dims);
[~, start_clear_norm] = clearance_metric_ws(clear_map, sx, sy, cell_m, public_vars);

free = inflated == 0;
deg = zeros(size(inflated));
ridge = false(size(inflated));
for y = 2:(ny - 1)
    for x = 2:(nx - 1)
        if ~free(y, x)
            continue;
        end
        deg(y, x) = free(y - 1, x) + free(y + 1, x) + free(y, x - 1) + free(y, x + 1);
        ridge(y, x) = voronoi_ridge_cell_ws(clear_map, x, y, public_vars);
    end
end

candidates = zeros(0, 14);
visited = zeros(0, 2);
if isfield(public_vars.disambiguate, 'visited_goals') && ~isempty(public_vars.disambiguate.visited_goals)
    visited = public_vars.disambiguate.visited_goals;
end
visited_sectors = getfield_with_default(public_vars.disambiguate, 'visited_sectors', []);
sector_width = getfield_with_default(public_vars.disambiguate, 'sector_width_rad', 45 * pi / 180);
sector_penalty = getfield_with_default(public_vars.disambiguate, 'sector_revisit_penalty', 1.25);

for y = 2:(ny - 1)
    for x = 2:(nx - 1)
        if ~free(y, x)
            continue;
        end
        is_junction = deg(y, x) >= 3;
        is_ridge = ridge(y, x);
        if ~is_junction && ~is_ridge
            continue;
        end
        xy = discrete_to_world_ws(x, y, limits, dims);
        d = norm(xy - public_vars.estimated_pose(1:2));
        if d < public_vars.disambiguate.min_goal_distance_m || d > public_vars.disambiguate.max_goal_distance_m
            continue;
        end
        if ~isempty(visited)
            dv = vecnorm(visited - xy, 2, 2);
            if any(dv < 0.9)
                continue;
            end
        end
        heading = atan2(xy(2) - public_vars.estimated_pose(2), xy(1) - public_vars.estimated_pose(1));
        sector_id = heading_sector_id_ws(heading, sector_width);
        sector_hits = sum(visited_sectors == sector_id);
        corridor_score = abs(double(x - sx)) + abs(double(y - sy));
        revisit_penalty = sector_penalty * sector_hits;
        [goal_clear, goal_clear_norm] = clearance_metric_ws(clear_map, x, y, cell_m, public_vars);
        if is_ridge && goal_clear < getfield_with_default(public_vars.disambiguate, 'voronoi_ridge_min_clearance_m', 0.38)
            continue;
        end
        radial_progress = max(0, d - 0.35 * abs(double(x - sx) + double(y - sy)) * cell_m);
        clearance_gain = getfield_with_default(public_vars.disambiguate, 'clearance_bias_gain', 1.0);
        ridge_bonus = getfield_with_default(public_vars.disambiguate, 'voronoi_ridge_gain', 0.95) * double(is_ridge);
        score = corridor_score ...
            - clearance_gain * goal_clear_norm ...
            - 0.18 * min(radial_progress / max(public_vars.disambiguate.max_goal_distance_m, 1e-6), 1.0) ...
            - ridge_bonus ...
            + revisit_penalty;
        candidates(end + 1, :) = [xy, d, score, sector_id, sector_hits, heading, revisit_penalty, goal_clear, goal_clear_norm, radial_progress, start_clear_norm, corridor_score, double(is_ridge)]; %#ok<AGROW>
    end
end

if isempty(candidates)
    return;
end

[~, order] = sortrows([candidates(:, 4), candidates(:, 3), -candidates(:, 9), candidates(:, 6)], [1, 2, 3, 4]);
candidates = candidates(order, :);

for i = 1:min(20, size(candidates, 1))
    xy = candidates(i, 1:2);
    tmp_ro = read_only_vars;
    tmp_ro.map.goal = xy;
    tmp_pub = public_vars;
    tmp_pub.path = [];
    tmp_pub.replan_path = true;
    path = plan_path(tmp_ro, tmp_pub);
    if ~isempty(path) && size(path, 1) >= 2
        info = evaluate_path_quality_ws(path, path, read_only_vars, tmp_pub);
        path_len = sum(vecnorm(diff(path(:, 1:2), 1, 1), 2, 2));
        clearance_norm = max(candidates(i, 9), getfield_with_default(info, 'min_clearance_m', 0) ...
            / max(getfield_with_default(public_vars.disambiguate, 'clearance_target_m', 0.45), 1e-6));
        direct_dist = max(candidates(i, 3), 1e-6);
        goal_detour = 0;
        if verify_selecting && all(isfinite(final_goal_xy)) && isfinite(current_goal_dist)
            candidate_goal_dist = norm(xy - final_goal_xy);
            goal_detour = candidate_goal_dist - current_goal_dist;
            if goal_detour > getfield_with_default(public_vars.verify, 'max_goal_distance_increase_m', 0.55)
                candidates(i, 4) = inf;
                continue;
            end
        end
        combined_score = 0.70 * getfield_with_default(info, 'score', 0) ...
            + getfield_with_default(public_vars.disambiguate, 'goal_path_clearance_gain', 0.65) * clearance_norm ...
            - getfield_with_default(public_vars.disambiguate, 'goal_path_turn_gain', 0.18) ...
              * max(0, (getfield_with_default(info, 'max_turn_deg', 0) - 35) / 60) ...
            - getfield_with_default(public_vars.disambiguate, 'goal_length_penalty', 0.12) ...
              * max(0, path_len / direct_dist - 1.25) ...
            + 0.18 * candidates(i, 13) ...
            - 0.05 * candidates(i, 7) ...
            - getfield_with_default(public_vars.verify, 'goal_distance_penalty', 0.28) * max(0, goal_detour);
        candidates(i, 4) = -combined_score;
    end
end

valid_rows = isfinite(candidates(:, 4)) & candidates(:, 4) < 0;
if ~any(valid_rows)
    return;
end
[~, best_idx] = min(candidates(valid_rows, 4));
valid_ix = find(valid_rows);
pick = valid_ix(best_idx);
goal_xy = candidates(pick, 1:2);
found = true;
end

function public_vars = register_disambiguation_sector(public_vars)
sector_id = getfield_with_default(public_vars.disambiguate, 'goal_sector_id', nan);
if isfinite(sector_id)
    visited = getfield_with_default(public_vars.disambiguate, 'visited_sectors', []);
    public_vars.disambiguate.visited_sectors = [visited(:); sector_id];
end
front = getfield_with_default(public_vars.startup_motion, 'last_front_open', nan);
left = getfield_with_default(public_vars.startup_motion, 'last_left_open', nan);
right = getfield_with_default(public_vars.startup_motion, 'last_right_open', nan);
heading = getfield_with_default(public_vars.startup_motion, 'last_best_heading', 0);
signature = string(corridor_signature_ws(front, left, right, heading));
visited_signatures = getfield_with_default(public_vars.disambiguate, 'visited_signatures', strings(0, 1));
public_vars.disambiguate.visited_signatures = [visited_signatures(:); signature];
public_vars.disambiguate.goal_sector_id = nan;
end

function sector_id = heading_sector_id_ws(heading, sector_width)
heading = wrap_to_pi_ws(heading);
sector_width = max(sector_width, 5 * pi / 180);
sector_id = floor((heading + pi) / sector_width) + 1;
end

function info = evaluate_path_quality_ws(path, raw_path, read_only_vars, public_vars)
info = struct('score', nan, 'min_clearance_m', nan, 'max_turn_deg', nan, ...
    'turn_density', nan, 'length_ratio', nan, 'planner_clearance_m', ...
    getfield_with_default(public_vars, 'planner_clearance_m', getfield_with_default(public_vars, 'path_clearance_m', 0.25)), ...
    'tracking_clearance_m', getfield_with_default(public_vars, 'tracking_clearance_m', getfield_with_default(public_vars, 'path_clearance_m', 0.25)), ...
    'mode', string(getfield_with_default(public_vars, 'path_smoothing_mode', "chaikin")));
if isempty(path) || size(path, 1) < 2
    return;
end

walls = [];
if isfield(read_only_vars, 'map') && isfield(read_only_vars.map, 'walls')
    walls = read_only_vars.map.walls;
end
if ~isempty(walls)
    info.min_clearance_m = path_min_clearance_ws(path, walls, 0.03);
end
seg = diff(path(:, 1:2), 1, 1);
seg_len = vecnorm(seg, 2, 2);
angles = atan2(seg(:, 2), seg(:, 1));
if numel(angles) >= 2
    dtheta = abs(wrap_to_pi_ws(diff(angles)));
    info.max_turn_deg = max(dtheta) * 180 / pi;
    total_len = max(sum(seg_len), 1e-6);
    info.turn_density = sum(dtheta) / total_len;
else
    info.max_turn_deg = 0;
    info.turn_density = 0;
end
if nargin >= 2 && ~isempty(raw_path) && size(raw_path, 1) >= 2
    raw_len = sum(vecnorm(diff(raw_path(:, 1:2), 1, 1), 2, 2));
else
    raw_len = sum(seg_len);
end
path_len = sum(seg_len);
info.length_ratio = path_len / max(raw_len, 1e-6);

score = 1.0;
if isfinite(info.min_clearance_m)
    clearance_target = max(0.10, info.tracking_clearance_m);
    score = score - 0.55 * max(0, 1 - info.min_clearance_m / clearance_target);
end
if isfinite(info.max_turn_deg)
    score = score - 0.25 * max(0, (info.max_turn_deg - 40) / 70);
end
if isfinite(info.turn_density)
    score = score - 0.20 * max(0, (info.turn_density - 0.45) / 0.70);
end
if isfinite(info.length_ratio)
    score = score - 0.20 * max(0, info.length_ratio - 1.10);
end
info.score = max(0, min(1, score));
end

function [clear_map, cell_m] = clearance_map_ws(grid, limits, dims)
cell_m = max((limits(2) - limits(1)) / max(dims(1), 1), (limits(4) - limits(3)) / max(dims(2), 1));
free = grid == 0;
[ny, nx] = size(grid);
clear_map = zeros(ny, nx);
obs_xy = [];
for y = 1:ny
    for x = 1:nx
        if ~free(y, x)
            obs_xy(end + 1, :) = [x, y]; %#ok<AGROW>
        end
    end
end
if isempty(obs_xy)
    clear_map(:) = inf;
    return;
end
for y = 1:ny
    for x = 1:nx
        if ~free(y, x)
            continue;
        end
        d2 = min(sum((obs_xy - [x, y]).^2, 2));
        clear_map(y, x) = sqrt(max(d2, 0)) * cell_m;
    end
end
end

function [clearance_m, clearance_norm] = clearance_metric_ws(clear_map, x, y, cell_m, public_vars)
clearance_m = 0;
clearance_norm = 0;
if isempty(clear_map)
    return;
end
[ny, nx] = size(clear_map);
x = min(max(round(x), 1), nx);
y = min(max(round(y), 1), ny);
clearance_m = clear_map(y, x);
if ~isfinite(clearance_m)
    clearance_m = 3.0 * cell_m;
end
target = getfield_with_default(public_vars.disambiguate, 'clearance_target_m', max(0.45, 1.7 * getfield_with_default(public_vars, 'tracking_clearance_m', 0.25)));
clearance_norm = max(0, min(clearance_m / max(target, 1e-6), 1.6));
end

function tf = voronoi_ridge_cell_ws(clear_map, x, y, public_vars)
tf = false;
if isempty(clear_map)
    return;
end
[ny, nx] = size(clear_map);
if x <= 1 || x >= nx || y <= 1 || y >= ny
    return;
end
c = clear_map(y, x);
if ~isfinite(c) || c <= 0
    return;
end
tol = getfield_with_default(public_vars.disambiguate, 'voronoi_ridge_neighbor_tol_m', 0.04);
n4 = [clear_map(y - 1, x), clear_map(y + 1, x), clear_map(y, x - 1), clear_map(y, x + 1)];
if any(~isfinite(n4))
    return;
end
dominates_ns = c >= n4(1) - tol && c >= n4(2) - tol;
dominates_ew = c >= n4(3) - tol && c >= n4(4) - tol;
ridge_span_ns = min(n4(1), n4(2)) >= 0.70 * c;
ridge_span_ew = min(n4(3), n4(4)) >= 0.70 * c;
tf = (dominates_ns && ridge_span_ew) || (dominates_ew && ridge_span_ns);
end

function score = local_pose_clearance_score_ws(pose, read_only_vars, public_vars)
score = 0;
if ~isfield(read_only_vars, 'map') || ~isfield(read_only_vars.map, 'walls') || isempty(read_only_vars.map.walls)
    return;
end
d = path_min_clearance_ws(pose(1:2), read_only_vars.map.walls, 0.02);
target = getfield_with_default(public_vars.disambiguate, 'clearance_target_m', max(0.45, 1.7 * getfield_with_default(public_vars, 'tracking_clearance_m', 0.25)));
score = max(0, min(d / max(target, 1e-6), 1.6));
end

function v = weighted_mean_ws(values, weights)
v = 0;
if isempty(values) || isempty(weights)
    return;
end
valid = isfinite(values(:)) & isfinite(weights(:));
if ~any(valid)
    return;
end
w = weights(valid);
w = w / max(sum(w), 1e-9);
v = sum(values(valid) .* w);
end

function dmin = path_min_clearance_ws(path, walls, ds)
if nargin < 3
    ds = 0.03;
end
pts = sample_path_ws(path, ds);
dmin = inf;
for i = 1:size(pts, 1)
    pi = pts(i, :);
    for w = 1:size(walls, 1)
        d = point_to_segment_distance_ws(pi, walls(w, 1:2), walls(w, 3:4));
        if d < dmin
            dmin = d;
        end
    end
end
end

function pts = sample_path_ws(path, ds)
pts = path(1, :);
for i = 1:(size(path, 1) - 1)
    seg = sample_segment_ws(path(i, :), path(i + 1, :), ds);
    if i > 1
        seg(1, :) = [];
    end
    pts = [pts; seg]; %#ok<AGROW>
end
end

function pts = sample_segment_ws(a, b, ds)
L = norm(b - a);
if L < 1e-12
    pts = a;
    return;
end
n = max(2, ceil(L / max(ds, 1e-3)) + 1);
t = linspace(0, 1, n)';
pts = (1 - t) .* a + t .* b;
end

function d = point_to_segment_distance_ws(p, a, b)
ab = b - a;
den = dot(ab, ab);
if den < 1e-12
    d = norm(p - a);
    return;
end
t = dot(p - a, ab) / den;
t = max(0, min(1, t));
proj = a + t * ab;
d = norm(p - proj);
end

function path = bridge_path_from_pose_ws(path, est_pose)
if isempty(path) || size(path, 1) < 2 || isempty(est_pose) || any(~isfinite(est_pose(1:2)))
    return;
end
start_xy = est_pose(1:2);
bridge_vec = path(1, 1:2) - start_xy;
bridge_len = norm(bridge_vec);
if bridge_len <= 0.32
    path(1, 1:2) = start_xy;
    return;
end
first_seg = path(2, 1:2) - path(1, 1:2);
first_seg_len = norm(first_seg);
if first_seg_len > 1e-6 && bridge_len <= 0.75
    cos_align = dot(bridge_vec, first_seg) / max(bridge_len * first_seg_len, 1e-9);
    if cos_align >= 0.75
        path = [start_xy; path];
        return;
    end
end
path(1, 1:2) = 0.65 * path(1, 1:2) + 0.35 * start_xy;
end

function clear_steps = get_ambiguity_clear_counter_ws(public_vars)
clear_steps = 0;
if isfield(public_vars, 'environment_ambiguity') ...
        && isfield(public_vars.environment_ambiguity, 'clear_counter') ...
        && isfinite(public_vars.environment_ambiguity.clear_counter)
    clear_steps = public_vars.environment_ambiguity.clear_counter;
elseif isfield(public_vars, 'ambiguity') ...
        && isfield(public_vars.ambiguity, 'clear_counter') ...
        && isfinite(public_vars.ambiguity.clear_counter)
    clear_steps = public_vars.ambiguity.clear_counter;
end
end

function [ix, iy] = continous_to_discrete_coords_ws(x, y, limits, dims)
nx = dims(1);
ny = dims(2);
if nx <= 1
    ix = 1;
else
    ix = round(1 + (x - limits(1)) * (nx - 1) / max(limits(3) - limits(1), 1e-9));
end
if ny <= 1
    iy = 1;
else
    iy = round(1 + (y - limits(2)) * (ny - 1) / max(limits(4) - limits(2), 1e-9));
end
end

function [ix, iy] = clip_to_grid_ws(ix, iy, nx, ny)
ix = max(1, min(nx, ix));
iy = max(1, min(ny, iy));
end

function [ix, iy] = snap_to_nearest_free_ws(grid, ix, iy)
[ny, nx] = size(grid);
[ix, iy] = clip_to_grid_ws(ix, iy, nx, ny);
if grid(iy, ix) == 0
    return;
end
max_r = max(nx, ny);
for r = 1:max_r
    for y = max(1, iy-r):min(ny, iy+r)
        for x = max(1, ix-r):min(nx, ix+r)
            if grid(y, x) == 0
                ix = x;
                iy = y;
                return;
            end
        end
    end
end
end

function grid_out = inflate_grid_ws(grid_in, limits, dims, clearance_m)
grid_out = grid_in;
if clearance_m <= 0
    return;
end
nx = dims(1);
ny = dims(2);
if nx <= 1 || ny <= 1
    return;
end
dx = (limits(3) - limits(1)) / (nx - 1);
dy = (limits(4) - limits(2)) / (ny - 1);
cell_m = max(dx, dy);
if ~isfinite(cell_m) || cell_m <= 0
    return;
end
r = ceil(clearance_m / cell_m);
if r <= 0
    return;
end
[yy, xx] = ndgrid(-r:r, -r:r);
se = (xx.^2 + yy.^2) <= r^2;
occ = grid_in ~= 0;
inflated = conv2(double(occ), double(se), 'same') > 0;
grid_out(inflated) = 1;
end

function c = lidar_match_cost(pred, meas)
pred = pred(:);
meas = meas(:);
valid = isfinite(pred) & isfinite(meas);
if ~any(valid)
    c = inf;
    return;
end
err = pred(valid) - meas(valid);
c = mean(err.^2);
mismatch = xor(isfinite(pred), isfinite(meas));
if any(mismatch)
    c = c + 0.12 * mean(mismatch);
end
end


function [front, left, right] = lidar_sector_minima(read_only_vars)
front = inf;
left = inf;
right = inf;
if ~isfield(read_only_vars, 'lidar_distances') || isempty(read_only_vars.lidar_distances)
    return;
end

d = read_only_vars.lidar_distances(:);
a = read_only_vars.lidar_config(:);
valid = isfinite(d) & isfinite(a);
d = d(valid);
a = a(valid);
if isempty(d)
    return;
end

a_wrap = wrap_to_pi_ws(a);
front_mask = abs(a_wrap) <= 35 * pi / 180;
left_mask = a_wrap > 20 * pi / 180 & a_wrap < 160 * pi / 180;
right_mask = a_wrap < -20 * pi / 180 & a_wrap > -160 * pi / 180;

if any(front_mask), front = min(d(front_mask)); end
if any(left_mask), left = min(d(left_mask)); end
if any(right_mask), right = min(d(right_mask)); end
end

function ambiguity = evaluate_environment_ambiguity(read_only_vars, public_vars)
ambiguity = struct('active', false, 'counter', 0, 'front', inf, 'left', inf, ...
    'right', inf, 'scan_change', inf);
[front, left, right] = lidar_sector_minima(read_only_vars);
ambiguity.front = front;
ambiguity.left = left;
ambiguity.right = right;

scan_change = inf;
if isfield(public_vars, 'lidar_history') && size(public_vars.lidar_history, 1) >= 1 ...
        && ~isempty(read_only_vars.lidar_distances)
    if size(public_vars.lidar_history, 1) >= 2
        prev = public_vars.lidar_history(end - 1, :);
    else
        prev = public_vars.lidar_history(end, :);
    end
    curr = read_only_vars.lidar_distances(:)';
    valid = isfinite(prev) & isfinite(curr);
    if any(valid)
        scan_change = mean(abs(curr(valid) - prev(valid)));
    end
end
ambiguity.scan_change = scan_change;

side_similar = isfinite(left) && isfinite(right) && abs(left - right) <= public_vars.ambiguity.side_similarity_m;
side_corridor = isfinite(left) && isfinite(right) ...
    && left >= public_vars.ambiguity.min_side_m && right >= public_vars.ambiguity.min_side_m ...
    && left <= public_vars.ambiguity.max_side_m && right <= public_vars.ambiguity.max_side_m;
front_open = isfinite(front) && front >= public_vars.ambiguity.corridor_front_m;
scan_static = isfinite(scan_change) && scan_change <= public_vars.ambiguity.scan_change_small;
side_ratio = inf;
if isfinite(left) && isfinite(right)
    side_ratio = max(left, right) / max(min(left, right), 1e-6);
end
weakly_symmetric = isfinite(side_ratio) && side_ratio <= 2.2;

ambiguity.active = front_open && side_corridor && scan_static && (side_similar || weakly_symmetric);
if isfield(public_vars, 'environment_ambiguity') && isfield(public_vars.environment_ambiguity, 'counter') ...
        && public_vars.environment_ambiguity.active && ambiguity.active
    ambiguity.counter = public_vars.environment_ambiguity.counter + 1;
elseif ambiguity.active
    ambiguity.counter = 1;
else
    ambiguity.counter = 0;
end
if ambiguity.active
    ambiguity.clear_counter = 0;
elseif isfield(public_vars, 'environment_ambiguity') && isfield(public_vars.environment_ambiguity, 'clear_counter')
    ambiguity.clear_counter = public_vars.environment_ambiguity.clear_counter + 1;
else
    ambiguity.clear_counter = 1;
end
end

function [xy_fused, weights] = fuse_xy(kf_xy, kf_var_xy, pf_xy, pf_var_xy)
kf_var_xy = max(kf_var_xy(:)', 1e-6);
pf_var_xy = max(pf_var_xy(:)', 1e-6);
wk = 1 ./ kf_var_xy;
wp = 1 ./ pf_var_xy;
den = wk + wp;
xy_fused = (wk .* kf_xy + wp .* pf_xy) ./ den;
weights = [wk ./ den; wp ./ den];
end

function [th_fused, weights] = fuse_theta(kf_th, kf_var_th, pf_th, pf_var_th)
kf_var_th = max(kf_var_th, 1e-6);
pf_var_th = max(pf_var_th, 1e-6);
wk = 1 / kf_var_th;
wp = 1 / pf_var_th;
den = wk + wp;
wk_n = wk / den;
wp_n = wp / den;
v = wk_n * [cos(kf_th), sin(kf_th)] + wp_n * [cos(pf_th), sin(pf_th)];
th_fused = atan2(v(2), v(1));
weights = [wk_n, wp_n];
end

function quality = evaluate_localization_quality(read_only_vars, public_vars, pf_valid, kf_valid, gnss_valid, lidar_valid)
quality = struct();
quality.speed_scale = 1.0;
quality.turn_scale = 1.0;
quality.stop_translation = false;
quality.reason = "ok";
quality.pf_cluster_radius_m = inf;
quality.pf_var_xy_mean = inf;
quality.kf_var_xy_mean = inf;
quality.disagreement_xy_m = inf;
quality.disagreement_theta_rad = inf;
quality.pose_valid = isfield(public_vars, 'estimated_pose') && ~isempty(public_vars.estimated_pose) ...
    && all(isfinite(public_vars.estimated_pose(1:3)));
quality.gnss_valid = gnss_valid;
quality.lidar_valid = lidar_valid;
quality.fusion_mode = "none";
quality.pf_dominant_mass = 0;
quality.pf_top_ratio = nan;
quality.pf_hypothesis_count = nan;
quality.scan_match_cost = inf;
quality.scan_match_ema = nan;
quality.path_distance_ema = nan;
quality.pf_cluster_ema = nan;
quality.disagreement_ema = nan;
quality.scan_match_trend = 0;
quality.path_distance_trend = 0;
quality.pf_cluster_trend = 0;
quality.disagreement_trend = 0;
quality.fusion_pf_score = nan;
quality.fusion_kf_score = nan;
quality.top_hypotheses = struct([]);
quality.h2_pose_sep_m = nan;
quality.h12_mass_ratio = nan;
quality.h12_scan_gap = nan;
quality.h12_stability_gap = nan;
for iHyp = 1:3
    quality.(sprintf('h%d_mass', iHyp)) = nan;
    quality.(sprintf('h%d_radius_m', iHyp)) = nan;
    quality.(sprintf('h%d_scan_score', iHyp)) = nan;
    quality.(sprintf('h%d_age', iHyp)) = 0;
    quality.(sprintf('h%d_stability', iHyp)) = 0;
end

if isfield(public_vars, 'fusion') && isfield(public_vars.fusion, 'mode') && ~isempty(public_vars.fusion.mode)
    quality.fusion_mode = string(public_vars.fusion.mode);
end
if isfield(public_vars, 'fusion') && isfield(public_vars.fusion, 'pf_score') && isfinite(public_vars.fusion.pf_score)
    quality.fusion_pf_score = public_vars.fusion.pf_score;
end
if isfield(public_vars, 'fusion') && isfield(public_vars.fusion, 'kf_score') && isfinite(public_vars.fusion.kf_score)
    quality.fusion_kf_score = public_vars.fusion.kf_score;
end

if ~quality.pose_valid
    quality.speed_scale = 0;
    quality.turn_scale = 0;
    quality.stop_translation = true;
    quality.reason = "invalid_pose";
    return;
end

if isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
    xy = public_vars.particles(:, 1:2);
    xy = xy(all(isfinite(xy), 2), :);
    if ~isempty(xy)
        center = median(xy, 1);
        quality.pf_cluster_radius_m = median(vecnorm(xy - center, 2, 2));
    end
    [pf_var_xy, ~] = pf_uncertainty(public_vars);
    quality.pf_var_xy_mean = mean(pf_var_xy);
end

if isfield(public_vars, 'pf_stats') && ~isempty(public_vars.pf_stats)
    if isfield(public_vars.pf_stats, 'dominant_mass') && isfinite(public_vars.pf_stats.dominant_mass)
        quality.pf_dominant_mass = public_vars.pf_stats.dominant_mass;
    end
    if isfield(public_vars.pf_stats, 'top_ratio') && isfinite(public_vars.pf_stats.top_ratio)
        quality.pf_top_ratio = public_vars.pf_stats.top_ratio;
    end
    if isfield(public_vars.pf_stats, 'hypothesis_count') && isfinite(public_vars.pf_stats.hypothesis_count)
        quality.pf_hypothesis_count = public_vars.pf_stats.hypothesis_count;
    end
    if isfield(public_vars.pf_stats, 'top_hypotheses') && ~isempty(public_vars.pf_stats.top_hypotheses)
        quality.top_hypotheses = public_vars.pf_stats.top_hypotheses;
        keep = min(3, numel(public_vars.pf_stats.top_hypotheses));
        for iHyp = 1:keep
            hyp = public_vars.pf_stats.top_hypotheses(iHyp);
            quality.(sprintf('h%d_mass', iHyp)) = getfield_with_default(hyp, 'mass', nan);
            quality.(sprintf('h%d_radius_m', iHyp)) = getfield_with_default(hyp, 'radius_m', nan);
            quality.(sprintf('h%d_scan_score', iHyp)) = getfield_with_default(hyp, 'scan_score', nan);
            quality.(sprintf('h%d_age', iHyp)) = getfield_with_default(hyp, 'age', 0);
            quality.(sprintf('h%d_stability', iHyp)) = getfield_with_default(hyp, 'stability', 0);
        end
        if keep >= 2
            h1_pose = getfield_with_default(public_vars.pf_stats.top_hypotheses(1), 'pose', [nan, nan, nan]);
            h2_pose = getfield_with_default(public_vars.pf_stats.top_hypotheses(2), 'pose', [nan, nan, nan]);
            if numel(h1_pose) >= 2 && numel(h2_pose) >= 2 ...
                    && all(isfinite(h1_pose(1:2))) && all(isfinite(h2_pose(1:2)))
                quality.h2_pose_sep_m = norm(h1_pose(1:2) - h2_pose(1:2));
            end
            if isfinite(quality.h1_mass) && isfinite(quality.h2_mass)
                quality.h12_mass_ratio = quality.h2_mass / max(quality.h1_mass, 1e-6);
            end
            if isfinite(quality.h1_scan_score) && isfinite(quality.h2_scan_score)
                quality.h12_scan_gap = quality.h1_scan_score - quality.h2_scan_score;
            end
            if isfinite(quality.h1_stability) && isfinite(quality.h2_stability)
                quality.h12_stability_gap = quality.h1_stability - quality.h2_stability;
            end
        end
    end
end

if isfield(public_vars, 'sigma') && ~isempty(public_vars.sigma) && all(isfinite(public_vars.sigma(:)))
    quality.kf_var_xy_mean = mean([public_vars.sigma(1,1), public_vars.sigma(2,2)]);
end

if pf_valid && kf_valid && isfield(public_vars, 'mu') && ~isempty(public_vars.mu)
    pf_pose = estimate_pose(public_vars);
    kf_pose = public_vars.mu(:)';
    if ~isempty(pf_pose) && all(isfinite(pf_pose))
        quality.disagreement_xy_m = norm(kf_pose(1:2) - pf_pose(1:2));
        quality.disagreement_theta_rad = abs(wrap_to_pi_ws(kf_pose(3) - pf_pose(3)));
    end
end

if lidar_valid && quality.pose_valid ...
        && isfield(read_only_vars, 'map') && ~isempty(read_only_vars.map) ...
        && isfield(read_only_vars, 'lidar_config') && ~isempty(read_only_vars.lidar_config) ...
        && isfield(read_only_vars, 'lidar_distances') && ~isempty(read_only_vars.lidar_distances)
    pred = compute_lidar_measurement(read_only_vars.map, public_vars.estimated_pose, read_only_vars.lidar_config);
    quality.scan_match_cost = lidar_match_cost(pred, read_only_vars.lidar_distances);
end

if strcmp(quality.fusion_mode, "none")
    quality.speed_scale = 0;
    quality.turn_scale = 0;
    quality.stop_translation = true;
    quality.reason = "no_fusion_mode";
    return;
end

if pf_valid && ~gnss_valid
    if quality.pf_cluster_radius_m > 1.0
        quality.speed_scale = 0.12;
        quality.turn_scale = 0.55;
        quality.reason = "pf_cluster_very_wide";
        return;
    elseif quality.pf_cluster_radius_m > 0.65
        quality.speed_scale = min(quality.speed_scale, 0.35);
        quality.turn_scale = min(quality.turn_scale, 0.75);
        quality.reason = "pf_cluster_wide";
    elseif quality.pf_cluster_radius_m > 0.45
        quality.speed_scale = min(quality.speed_scale, 0.60);
        quality.turn_scale = min(quality.turn_scale, 0.85);
        quality.reason = "pf_cluster_medium";
    end

    if quality.pf_dominant_mass < 0.48
        quality.speed_scale = min(quality.speed_scale, 0.18);
        quality.turn_scale = min(quality.turn_scale, 0.55);
        quality.reason = "pf_not_unique";
    elseif quality.pf_dominant_mass < 0.62
        quality.speed_scale = min(quality.speed_scale, 0.40);
        quality.turn_scale = min(quality.turn_scale, 0.75);
        quality.reason = "pf_weakly_unique";
    end

    if isfinite(quality.scan_match_cost)
        if quality.scan_match_cost > 0.35
            quality.speed_scale = 0.05;
            quality.turn_scale = 0.40;
            quality.stop_translation = true;
            quality.reason = "scan_match_very_bad";
            return;
        elseif quality.scan_match_cost > 0.22
            quality.speed_scale = min(quality.speed_scale, 0.20);
            quality.turn_scale = min(quality.turn_scale, 0.55);
            quality.reason = "scan_match_bad";
        elseif quality.scan_match_cost > 0.16
            quality.speed_scale = min(quality.speed_scale, 0.50);
            quality.turn_scale = min(quality.turn_scale, 0.75);
            quality.reason = "scan_match_warn";
        end
    end
end

if kf_valid && quality.kf_var_xy_mean > 0.8
    quality.speed_scale = min(quality.speed_scale, 0.28);
    quality.turn_scale = min(quality.turn_scale, 0.65);
    quality.reason = "kf_cov_high";
elseif kf_valid && quality.kf_var_xy_mean > 0.35
    quality.speed_scale = min(quality.speed_scale, 0.65);
    quality.turn_scale = min(quality.turn_scale, 0.85);
    quality.reason = "kf_cov_medium";
end

if isfinite(quality.disagreement_xy_m) && isfinite(quality.disagreement_theta_rad)
    if quality.disagreement_xy_m > 1.4 || quality.disagreement_theta_rad > 110 * pi / 180
        quality.speed_scale = 0.10;
        quality.turn_scale = 0.55;
        quality.reason = "fusion_disagreement_large";
        return;
    elseif quality.disagreement_xy_m > 0.8 || quality.disagreement_theta_rad > 60 * pi / 180
        quality.speed_scale = min(quality.speed_scale, 0.40);
        quality.turn_scale = min(quality.turn_scale, 0.75);
        quality.reason = "fusion_disagreement_medium";
    end
end
end

function [var_xy, var_th] = pf_uncertainty(public_vars)
var_xy = [1, 1];
var_th = 1;
if ~isfield(public_vars, 'particles') || isempty(public_vars.particles)
    return;
end
p = public_vars.particles;
valid = all(isfinite(p(:, 1:3)), 2);
p = p(valid, :);
w = [];
if isfield(public_vars, 'particle_weights') && numel(public_vars.particle_weights) >= size(public_vars.particles, 1)
    w = public_vars.particle_weights(valid);
end
if size(p, 1) < 5
    return;
end
if isempty(w) || any(~isfinite(w)) || sum(w) <= 0
    w = ones(size(p, 1), 1) / size(p, 1);
else
    w = w / sum(w);
end
mx = sum(w .* p(:, 1));
my = sum(w .* p(:, 2));
vx = sum(w .* (p(:, 1) - mx).^2);
vy = sum(w .* (p(:, 2) - my).^2);
var_xy = max([vx, vy], 1e-6);
c = sum(w .* cos(p(:, 3)));
s = sum(w .* sin(p(:, 3)));
R = sqrt(c^2 + s^2);
var_th = max(1 - R, 1e-3);
end

function a = wrap_to_pi_ws(a)
a = mod(a + pi, 2 * pi) - pi;
end
