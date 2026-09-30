#include <cmath>

#include <gtest/gtest.h>

#include "trajectory_mode_fw/vtp_guidance.hpp"

using namespace trajectory_mode_fw;

// Reference values computed with an independent Python port of PX4 FlightTaskTraj::update()
TEST(VtpGuidance, MatchesInternalModeClosing)
{
	ManagerOutput in;
	in.lambda = 0.20f; in.gamma_t = 0.35f; in.gamma_m = 0.15f;
	in.v_t = 16.f; in.v_m = 15.f; in.r = 16.f; in.a_t = 1.2f; in.j_t = 0.3f;
	GuidanceParams p;
	p.a_m_max = 100.f; // unsaturated
	const GuidanceOutput out = computeGuidance(in, p);
	EXPECT_TRUE(out.finite);
	EXPECT_NEAR(out.a_m, 1.59983f, 1e-3f);
	EXPECT_NEAR(out.a_long, 0.3f, 1e-6f);
	EXPECT_NEAR(out.s, 0.31987f, 1e-3f);
}

TEST(VtpGuidance, MatchesInternalModeOpening)
{
	ManagerOutput in;
	in.lambda = -0.1f; in.gamma_t = 0.1f; in.gamma_m = 0.4f;
	in.v_t = 16.f; in.v_m = 14.f; in.r = 15.5f; in.a_t = -0.8f; in.j_t = -0.2f;
	GuidanceParams p;
	p.a_m_max = 100.f;
	const GuidanceOutput out = computeGuidance(in, p);
	EXPECT_NEAR(out.a_m, -3.86621f, 1e-3f);
	EXPECT_NEAR(out.s, -0.96687f, 1e-3f);

	p.a_m_max = 2.f; // TRAJ_A_M_MAX clamp
	EXPECT_FLOAT_EQ(computeGuidance(in, p).a_m, -2.f);
}

namespace
{
Trajectory straightNorth(int n)
{
	Trajectory t;
	t.id = 1;

	for (int i = 0; i < n; ++i) {
		t.points.push_back({0.5f * i, 0.f, 16.f, 0.f, 0.f, 0.f, 0.5f * i / 16.f});
	}

	return t;
}
} // namespace

TEST(TrajectoryManager, ApproachesFirstPointThenAdvances)
{
	TrajectoryManager m;
	m.load(straightNorth(400));

	// Far behind the start: the target is waypoint 0 at the actual distance
	ManagerOutput o = m.update({-40.f, 0.f}, {16.f, 0.f}, 15.f, 100);
	EXPECT_EQ(o.index, 0u);
	EXPECT_NEAR(o.r, 40.f, 1e-4f);

	// Within r* of waypoint 0: the closest point >= r* ahead becomes the target
	o = m.update({-10.f, 0.f}, {16.f, 0.f}, 15.f, 100);
	EXPECT_EQ(o.index, 10u); // 5 m along the path = 15 m from the vehicle
	EXPECT_NEAR(o.r, 15.f, 1e-4f);
	EXPECT_NEAR(o.lambda, 0.f, 1e-6f);
}

TEST(TrajectoryManager, NoTargetInRangeAtTheEnd)
{
	// Same behaviour as PX4: no point >= r* in the window -> r = INFINITY and the index falls back to 0
	TrajectoryManager m;
	m.load(straightNorth(40)); // 19.5 m long
	// 10 m along: every point is closer than r* = 15 m (the farthest is 10 m back to waypoint 0)
	ManagerOutput o = m.update({10.f, 0.f}, {16.f, 0.f}, 15.f, 100);
	EXPECT_TRUE(std::isinf(o.r));
	EXPECT_EQ(o.index, 0u);
	EXPECT_FALSE(computeGuidance(o, GuidanceParams{}).finite);
}

TEST(TrajectoryManager, MirrorSymmetric)
{
	Trajectory right, left;

	for (int i = 0; i < 300; ++i) {
		const float s = 0.5f * i, k = 1.f / 60.f; // arc of radius 60 m
		const float th = s * k;
		right.points.push_back({60.f * std::sin(th), 60.f * (1.f - std::cos(th)), 16.f * std::cos(th),
					16.f * std::sin(th), 16.f * 16.f * k, 0.f, s / 16.f});
		const TrajectoryPoint &p = right.points.back();
		left.points.push_back({p.x, -p.y, p.vx, -p.vy, -p.at, 0.f, p.t});
	}

	TrajectoryManager mr, ml;
	mr.load(right);
	ml.load(left);
	const ManagerOutput orr = mr.update({-5.f, 1.f}, {15.f, 1.f}, 15.f, 100);
	const ManagerOutput ol = ml.update({-5.f, -1.f}, {15.f, -1.f}, 15.f, 100);
	const GuidanceOutput gr = computeGuidance(orr, GuidanceParams{});
	const GuidanceOutput gl = computeGuidance(ol, GuidanceParams{});
	EXPECT_EQ(orr.index, ol.index);
	EXPECT_NEAR(gr.a_m, -gl.a_m, 1e-4f);
	EXPECT_NEAR(gr.a_long, gl.a_long, 1e-6f);
}

#include "trajectory_mode_fw/yaml_params.hpp"

TEST(YamlParams, UpdatesValuesKeepsCommentsAndOtherKeys)
{
	const std::string in =
		"# header comment\n"
		"trajectory_mode_fw:\n"
		"  ros__parameters:\n"
		"    traj_r_star: 15.0        # [m]\n"
		"    traj_smc_beta: 0.12\n"
		"    mqtt_port: 1883\n";
	const std::string out = updateYamlValues(in, {{"traj_r_star", yamlDouble(30)}, {"traj_smc_beta", yamlDouble(0.25)}});
	EXPECT_EQ(out,
		  "# header comment\n"
		  "trajectory_mode_fw:\n"
		  "  ros__parameters:\n"
		  "    traj_r_star: 30.0        # [m]\n"
		  "    traj_smc_beta: 0.25\n"
		  "    mqtt_port: 1883\n");
}

TEST(YamlParams, AppendsMissingKeyUnderRosParameters)
{
	const std::string in = "node:\n  ros__parameters:\n    a: 1.0\n";
	EXPECT_EQ(updateYamlValues(in, {{"traj_a_m_max", yamlDouble(4)}}),
		  "node:\n  ros__parameters:\n    traj_a_m_max: 4.0\n    a: 1.0\n");
}

TEST(YamlParams, DoublesStayDoubles)
{
	EXPECT_EQ(yamlDouble(30), "30.0");
	EXPECT_EQ(yamlDouble(0.05), "0.05");
	EXPECT_EQ(yamlDouble(1.4), "1.4");
}

#include "trajectory_mode_fw/trajectory_upload.hpp"

TEST(TrajectoryUpload, CompleteAfterAllIndicesInAnyOrder)
{
	TrajectoryUpload upload;
	EXPECT_FALSE(upload.add(0, {}).has_value()); // before INITIATE: ignored

	upload.initiate(7, 3);
	EXPECT_FALSE(upload.add(2, {2.f, 0.f, 16.f, 0.f, 0.f, 0.f, 0.f}).has_value());
	EXPECT_FALSE(upload.add(0, {0.f, 0.f, 16.f, 0.f, 0.f, 0.f, 0.f}).has_value());
	EXPECT_FALSE(upload.add(0, {}).has_value()); // duplicate
	EXPECT_FALSE(upload.add(5, {}).has_value()); // out of range

	const auto trajectory = upload.add(1, {1.f, 0.f, 16.f, 0.f, 0.f, 0.f, 0.f});
	ASSERT_TRUE(trajectory.has_value());
	EXPECT_EQ(trajectory->id, 7);
	ASSERT_EQ(trajectory->points.size(), 3u);

	for (int i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(trajectory->points[i].x, static_cast<float>(i));
	}

	EXPECT_FALSE(upload.add(1, {}).has_value()); // finished: ignored until the next INITIATE
}

TEST(TrajectoryUpload, InitiateRestarts)
{
	TrajectoryUpload upload;
	upload.initiate(1, 2);
	upload.add(0, {});
	upload.initiate(2, 1);
	const auto trajectory = upload.add(0, {5.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
	ASSERT_TRUE(trajectory.has_value());
	EXPECT_EQ(trajectory->id, 2);
	EXPECT_EQ(trajectory->points.size(), 1u);
}

#include "fw_sim.hpp"

// Fixed-wing closed loop (fw_sim.hpp): acquisition + the internal mode's guidance must bring the vehicle onto the
// path from any start and follow it to the end
namespace
{

struct StartCase {
	float x, y, course;
};

constexpr StartCase kStarts[] = {
	{0.f, 400.f, 0.f},                          // 400 m beside the start, heading along
	{0.f, 0.f, float(M_PI)},                    // on the first point, flying the wrong way
	{-1000.f, -300.f, float(M_PI)},             // 1 km away, flying away
	{150.f, -200.f, float(M_PI_2)},             // beside the path, flying across it
	{100.f, 5.f, 0.f},                          // on the path ahead of the start, aligned
};

void expectFollows(const FwControllerParams &params, float max_mean_error)
{
	const Trajectory path = sim::makePath(0.f, 0.f, 0.f);

	for (const StartCase &start : kStarts) {
		sim::Aircraft aircraft;
		aircraft.x = start.x;
		aircraft.y = start.y;
		aircraft.course = start.course;
		const sim::Result r = sim::fly(aircraft, path, params);
		SCOPED_TRACE(testing::Message() << "start " << start.x << ", " << start.y << ", course " << start.course);
		EXPECT_TRUE(r.captured);
		EXPECT_TRUE(r.finished);
		EXPECT_LT(r.mean_error, max_mean_error);
	}
}

} // namespace

TEST(FwTrajectoryController, FollowsPathFromAnyStartDefaultGains)
{
	FwControllerParams params;
	params.guidance.r_star = 30.f;
	expectFollows(params, 10.f);
}

TEST(FwTrajectoryController, FollowsPathFromAnyStartAggressiveGains)
{
	FwControllerParams params;
	params.guidance.r_star = 30.f;
	params.guidance.beta = 0.5f;
	params.guidance.epsilon = 0.1f;
	params.guidance.a_m_max = 8.f;
	params.guidance.a_long_max = 2.f;
	expectFollows(params, 20.f);
}

TEST(PathAcquisition, NoCaptureWhileFlyingAgainstThePath)
{
	PathAcquisition acquisition;
	const TrajectoryPoint point{0.f, 0.f, 16.f, 0.f, 0.f, 0.f, 0.f};
	EXPECT_FALSE(acquisition.update({1.f, 0.f}, {-16.f, 0.f}, point, 30.f, 4.f).captured);
	EXPECT_TRUE(acquisition.update({1.f, 0.f}, {16.f, 0.f}, point, 30.f, 4.f).captured);
}
