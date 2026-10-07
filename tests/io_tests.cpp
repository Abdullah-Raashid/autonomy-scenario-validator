#include "io/csv_reader.h"
#include "io/numeric.h"
#include "io/scenario_loader.h"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

namespace {
class CsvInput : public testing::Test {
  protected:
    std::filesystem::path dir;
    void SetUp() override {
        static std::atomic<unsigned> counter{0};
        dir = std::filesystem::temp_directory_path() /
              ("asv-test-" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
               std::to_string(counter++));
        std::filesystem::create_directories(dir);
    }
    void TearDown() override { std::filesystem::remove_all(dir); }
    std::string Write(const std::string &name, const std::string &text) {
        const auto p = dir / name;
        std::ofstream(p) << text;
        return p.string();
    }
    bool Load(const std::string &trajectory, const std::string &obstacles, asv::Scenario &s,
              std::string &err) {
        return asv::ScenarioLoader::LoadFromCsv(Write("traj.csv", trajectory),
                                                Write("obs.csv", obstacles), s, err);
    }
};
} // namespace
TEST(Numeric, StrictFiniteParsing) {
    EXPECT_DOUBLE_EQ(asv::ParseFiniteDouble(" 1.25e2 "), 125);
    for (const auto *v : {"", "nan", "inf", "1oops", "1e999", "1 2", "--1", "0x10"})
        EXPECT_THROW(asv::ParseFiniteDouble(v), std::invalid_argument) << v;
}
TEST_F(CsvInput, CsvQuotedCommasEscapedQuotesCrlfAndTrailingField) {
    std::vector<std::vector<std::string>> rows;
    std::string err;
    ASSERT_TRUE(asv::CsvReader::ReadAll(
        Write("csv", "\xef\xbb\xbf id,x,y\r\n\n\"a,b\",\"say \"\"hi\"\"\",\r\n"), rows, err));
    ASSERT_EQ(rows.size(), 2U);
    EXPECT_EQ(rows[0][0], "id");
    EXPECT_EQ(rows[1][0], "a,b");
    EXPECT_EQ(rows[1][1], "say \"hi\"");
    EXPECT_EQ(rows[1][2], "");
}
TEST_F(CsvInput, CsvMalformedQuoteAndMissingFilePreserveOutput) {
    std::vector<std::vector<std::string>> rows = {{"original"}};
    std::string err;
    for (const auto *bad : {"\"unterminated\n", "ab\"cd,1\n", "\"ok\"junk,1\n"}) {
        EXPECT_FALSE(asv::CsvReader::ReadAll(Write("bad", bad), rows, err));
        EXPECT_EQ(rows[0][0], "original");
        EXPECT_NE(err.find("line 1"), std::string::npos);
    }
    EXPECT_FALSE(asv::CsvReader::ReadAll((dir / "missing").string(), rows, err));
}
TEST_F(CsvInput, LegacySchemaAndOptionalNoObstacles) {
    asv::Scenario s;
    std::string err;
    ASSERT_TRUE(Load("t,x,y\n0,0,0\n1,2,0\n", "id,cx,cy,w,h\ncone,20,0,1,1\n", s, err));
    EXPECT_EQ(s.ego.points.size(), 2U);
    EXPECT_EQ(s.obstacles.items.size(), 1U);
    EXPECT_FALSE(s.ego.points[0].yaw);
    ASSERT_TRUE(
        asv::ScenarioLoader::LoadFromCsv(Write("traj", "t,x,y\n0,0,0\n1,1,0\n"), "", s, err));
    EXPECT_TRUE(s.obstacles.items.empty());
}
TEST_F(CsvInput, DynamicSchemaAndHeading) {
    asv::Scenario s;
    std::string err;
    ASSERT_TRUE(Load("t,x,y,yaw\n0,0,0,0.25\n1,2,0,0.5\n",
                     "id,cx,cy,w,h,yaw,vx,vy\n\"a,b\",20,0,4,2,0.2,1,-2\n", s, err));
    EXPECT_DOUBLE_EQ(*s.ego.points[0].yaw, 0.25);
    EXPECT_DOUBLE_EQ(s.obstacles.items[0].velocity.y, -2);
    EXPECT_EQ(s.obstacles.items[0].id, "a,b");
}
TEST_F(CsvInput, EmptyObstaclesHeaderIsValid) {
    asv::Scenario s;
    std::string err;
    ASSERT_TRUE(Load("t,x,y\n0,0,0\n1,1,0\n", "id,cx,cy,w,h\n", s, err));
    EXPECT_TRUE(s.obstacles.items.empty());
}
TEST_F(CsvInput, BadTrajectoryRowsDoNotPartiallyMutateScenario) {
    const char *cases[] = {"",
                           "t,x,y\n",
                           "time,x,y\n0,0,0\n1,1,0\n",
                           "t,x,y\n0,0,0\n1,1\n",
                           "t,x,y\n0,0,0\n1,2,3,4\n",
                           "t,x,y\n0,0,0\n0,1,0\n",
                           "t,x,y\n1,0,0\n0,1,0\n",
                           "t,x,y\n-1,0,0\n1,1,0\n",
                           "t,x,y\n0,0,0\n1,nan,0\n",
                           "t,x,y\n0,0,0\n1,oops,0\n",
                           "t,x,y\n0,0,0\n1,1,\n"};
    for (const auto *text : cases) {
        asv::Scenario s;
        s.ego.points = {{42, 7, 8, {}}};
        std::string err;
        EXPECT_FALSE(Load(text, "id,cx,cy,w,h\n", s, err)) << text;
        EXPECT_EQ(s.ego.points.size(), 1U);
        EXPECT_DOUBLE_EQ(s.ego.points[0].t, 42);
        EXPECT_FALSE(err.empty());
    }
}
TEST_F(CsvInput, InvalidObstacleRowsRejected) {
    for (const auto *text :
         {"", "id,cx,cy,w,h\na,1,2,0,1\n", "id,cx,cy,w,h\na,1,2,-1,1\n", "id,cx,cy,w,h\n,1,2,1,1\n",
          "id,cx,cy,w,h\na,1,2,1,1\na,2,3,1,1\n", "id,cx,cy,w,h\na,1,2,1\n"}) {
        asv::Scenario s;
        std::string err;
        EXPECT_FALSE(Load("t,x,y\n0,0,0\n1,1,0\n", text, s, err));
    }
}

TEST_F(CsvInput, DerivedErrorsDoNotBlameAnObstacleRow) {
    asv::Scenario s;
    std::string err;
    EXPECT_FALSE(asv::ScenarioLoader::LoadFromCsv(
        Write("overflow.csv", "t,x,y\n0,-1e308,0\n1,1e308,0\n"), "", s, err));
    EXPECT_EQ(err.find("scenario consistency: "), 0U);
}
