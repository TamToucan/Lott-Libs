#include <cmath>
#include <iostream>
#include <vector>

#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationGraph.hpp"
#include "Router.hpp"
// #define USE_CUTE_CAVE
#ifndef USE_CUTE_CAVE
#include "core/Cave.h"
#else
#include "cute/CuteCave.hpp"
#endif
std::pair<float, float> computeDirection(float angleDeg) {
  const double MYPI = 3.14159265358979323846;
  double radians = angleDeg * (MYPI / 180.0);
  return {std::cos(radians), std::sin(radians)};
}

Cave::TileMap makeCave() {
  // Generate the cave
  Cave::TileMap tileMap;
#ifndef USE_CUTE_CAVE
  Cave::CaveInfo info;
  Cave::GenerationParams params;
  // Generation parameters
  params.seed = 424242;
  params.mOctaves = 1;
  params.mPerlin = false;
  params.mWallChance = 0.65;
  params.mFreq = 13.7;

  Cave::GenerationStep step;
  step.b3_min = 3;
  step.b3_max = 4;
  step.b5_min = 12;
  step.b5_max = 16;
  step.s3_min = 2;
  step.s3_max = 5;
  step.s5_min = 10;
  step.s5_max = 14;
  step.reps = 2;
  params.mGenerations.push_back(step);

  // CaveInfo parameters
  info.mCaveWidth = 32;
  info.mCaveHeight = 32;
  info.mBorderWidth = 1;
  info.mBorderHeight = 1;
  info.mCellWidth = 8;
  info.mCellHeight = 8;
  Cave::Cave cave(info, params);
  tileMap = cave.generate();

#else

  // Cave generation params
  Cave::GenerationStep step;
  step.b3_min = 3;
  step.b3_max = 4;
  step.b5_min = 12;
  step.b5_max = 16;
  step.s3_min = 2;
  step.s3_max = 5;
  step.s5_min = 10;
  step.s5_max = 14;
  step.reps = 2;
  std::vector<Cave::GenerationStep> gens;
  gens.push_back(step);

  // Setup Params
  CuteCave::CuteCave cave;
  cave.setCaveSize(50, 50)
      .setSmoothing(true)
      .setSmoothCorners(true)
      .setSmoothPoints(true)
      .setBorderCellSize(1, 1)
      .setCellSize(1, 1)
      .setAmp(1.0f)
      .setOctaves(1)
      .setPerlin(false)
      .setWallChance(0.65f)
      .setFreq(13.7f)
      .setGenerations(gens);
  tileMap = cave.make_cave(4242);
#endif

  // Print the tile map to the console
  for (int y = 0; y < tileMap.size(); ++y) {
    for (int x = 0; x < tileMap[0].size(); ++x) {
      std::cout << ((tileMap[y][x] == Cave::FLOOR) ? ' ' : '#');
    }
    std::cout << std::endl;
  }

  return tileMap;
}

int main() {
  auto tileMap = makeCave();

  // Convert to DistanceMap format (0=WALL, 1=PATH)
  auto grid = tileMap;
  for (auto& row : grid) {
    for (auto& cell : row) {
      cell = (cell == Cave::FLOOR) ? 1 : 0;
    }
  }

  auto graph = DistanceMap::GridToGraph::makeGraph(grid);
  auto pathGrid(grid);
  DistanceMap::Router::Info info;
  info.mCaveHeight = 32;
  info.mCellWidth = 8;
  info.mCellHeight = 8;
  DistanceMap::Routing::NavigationGraph navGraph;
  navGraph.initialize(graph, info);
  DistanceMap::GridType::Vec2 from(300, 250);
  DistanceMap::GridType::Vec2 to(1950, 1086);
  DistanceMap::Router::RouteCtx* ctx = new DistanceMap::Router::RouteCtx();
  ctx->type = -1;
  int count = 2000;
  int mv = 1;
  bool reached_target = false;
  DistanceMap::GridType::Point toPnt = {(int)(to.x / (info.mCellWidth * 8)),
                                        (int)(to.y / (info.mCellHeight * 8))};
  DistanceMap::GridType::Point prevPnt = toPnt;
#if 1
  do {
    DistanceMap::GridType::Point fromPnt = {
        (int)(from.x / (info.mCellWidth * 8)),
        (int)(from.y / (info.mCellHeight * 8))};
    reached_target =
        (fromPnt.first == toPnt.first && fromPnt.second == toPnt.second);
    /*
if (prevPnt != fromPnt) {
  if (pathGrid[fromPnt.second][fromPnt.first] == 'x') {
    std::cerr << "ERROR: LOOPED BACK TO " << fromPnt.first << ","
              << fromPnt.second << std::endl;
    break;
  }
}
*/
    prevPnt = fromPnt;
    if (pathGrid[fromPnt.second][fromPnt.first] == 0) {
      std::cerr << "ERROR: WALL " << fromPnt.first << "," << fromPnt.second
                << std::endl;
      break;
    }
    pathGrid[fromPnt.second][fromPnt.first] = 'x';
    std::cerr << "MOVEFROM: " << fromPnt.first << "," << fromPnt.second
              << std::endl;

    float ang = navGraph.getMoveDirection(ctx, from, to, 0);
    // std::pair<float, float> mv = computeDirection(ang); // Not needed for resolveMove

    // Resolve move with sliding
    DistanceMap::GridType::Vec2 nextPos = navGraph.resolveMove(from, ang, 13);

    // Calculate effective movement for logging
    float movedX = nextPos.x - from.x;
    float movedY = nextPos.y - from.y;

    from = nextPos;
    DistanceMap::GridType::Point nw = {(int)(from.x / (info.mCellWidth * 8)),
                                       (int)(from.y / (info.mCellHeight * 8))};
    std::cerr << "CTV MV " << movedX << "," << movedY << "  ang " << ang
              << " cell: " << fromPnt.first << "," << fromPnt.second << " -> "
              << nw.first << "," << nw.second << std::endl;

  } while (!reached_target && (--count > 0));
  delete ctx;
#endif
  for (int row = 0; row < pathGrid.size(); ++row) {
    for (int col = 0; col < pathGrid[0].size(); ++col) {
      int v = pathGrid[row][col];
      if (toPnt.first == col && toPnt.second == row) {
        std::cerr << "T";
      } else if (v == 0) {
        std::cerr << "#";
      } else if (v == 'x') {
        std::cerr << "x";
      } else {
        std::cerr << " ";
      }
    }
    std::cerr << std::endl;
  }

  if (reached_target) {
    std::cerr << "OK: PATH FOUND" << std::endl;
  } else {
    std::cerr << "ERROR: NO PATH" << std::endl;
  }

  return reached_target ? 0 : 1;
}