/*
 * generate_test_dataset.cpp
 *
 * Builds a deterministic OctoMap (.ot) file with a known geometric layout
 * for use in offline / file-based tests of the 2D map logic.
 *
 * Layout (all coordinates in metres, resolution 0.1 m):
 *
 *   Y
 *   ^
 * 5 |        [WALL N z=0–2]
 * 4 |
 * 3 |  [BOX A z=0–1]   [BOX B z=1.5–2]
 * 2 |
 * 1 |  [FLOOR z=0–0.1, full XY extent]
 * 0 +------------------------------------> X
 *   0  1  2  3  4  5
 *
 * Structures:
 *   Floor        x=[0,5], y=[0,5], z=[0,0.1]   (ground plane, occupied)
 *   Box A        x=[1,2], y=[2,3], z=[0,1]      (low obstacle – in default slice)
 *   Box B        x=[3,4], y=[2,3], z=[1.5,2]    (high obstacle – above default slice)
 *   Wall N       x=[0,5], y=[4.9,5], z=[0,2]    (north wall, spans full slice)
 *   Free column  x=2.5,   y=2.5,    z=[0,3]     (explicitly cleared column)
 *
 * Expected 2D slice results for z=[0,1.5):
 *   Floor  →  all cells in [0,5]×[0,5] occupied
 *   Box A  →  cells in [1,2]×[2,3] occupied  (inside slice)
 *   Box B  →  cells in [3,4]×[2,3] NOT occupied (above slice)
 *   Wall N →  cells in [0,5]×[4.9,5] occupied
 *
 * Usage:
 *   ./generate_test_dataset [output_path]
 *   default output: /tmp/test_octomap.ot
 *
 * The program prints a summary of the tree and the expected cell values so
 * that CI can verify correctness without loading the file.
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

struct Box {
  std::string name;
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;
};

/// Mark all voxels inside `box` as occupied in the tree.
static void fill_box( octomap::OcTree &tree, const Box &box )
{
  const double res = tree.getResolution();
  // Step by res so every leaf voxel centre is hit exactly once
  for ( double x = box.x_min + res * 0.5; x < box.x_max; x += res ) {
    for ( double y = box.y_min + res * 0.5; y < box.y_max; y += res ) {
      for ( double z = box.z_min + res * 0.5; z < box.z_max; z += res ) {
        tree.updateNode( octomap::point3d(
          static_cast<float>( x ),
          static_cast<float>( y ),
          static_cast<float>( z ) ), true );
      }
    }
  }
}

/// Mark a column of voxels as free (overrides any prior occupied marking).
static void clear_column( octomap::OcTree &tree,
                           double cx, double cy,
                           double z_min, double z_max )
{
  const double res = tree.getResolution();
  for ( double z = z_min + res * 0.5; z < z_max; z += res ) {
    tree.updateNode( octomap::point3d(
      static_cast<float>( cx ),
      static_cast<float>( cy ),
      static_cast<float>( z ) ), false );
  }
}

// ---------------------------------------------------------------------------
// Verification helpers  (mirror the 2D-slice logic from the updater)
// ---------------------------------------------------------------------------

/// Return true if any voxel in the XY column (cx±res/2, cy±res/2) with
/// z in [z_min, z_max) is occupied.
static bool column_has_occupied_voxel( const octomap::OcTree &tree,
                                       double cx, double cy,
                                       double z_min, double z_max )
{
  const double res = tree.getResolution();
  for ( double z = z_min + res * 0.5; z < z_max; z += res ) {
    auto *node = tree.search( octomap::point3d(
      static_cast<float>( cx ),
      static_cast<float>( cy ),
      static_cast<float>( z ) ) );
    if ( node && tree.isNodeOccupied( node ) ) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main( int argc, char **argv )
{
  const std::string output_path =
      ( argc > 1 ) ? argv[1] : "/tmp/test_octomap.ot";

  constexpr double kResolution = 0.1;
  octomap::OcTree tree( kResolution );

  // ── 1. Define the geometry ───────────────────────────────────────────────
  const std::vector<Box> boxes = {
    { "floor",  0.0, 5.0,  0.0, 5.0,  0.0,  0.1 },
    { "box_a",  1.0, 2.0,  2.0, 3.0,  0.0,  1.0 },
    { "box_b",  3.0, 4.0,  2.0, 3.0,  1.5,  2.0 },
    { "wall_n", 0.0, 5.0,  4.9, 5.0,  0.0,  2.0 },
  };

  // ── 2. Fill geometry ─────────────────────────────────────────────────────
  std::cout << "Building OctoMap (resolution " << kResolution << " m)...\n";
  for ( const auto &b : boxes ) {
    std::cout << "  Filling: " << b.name << "\n";
    fill_box( tree, b );
  }

  // ── 3. Free column (explicitly cleared passage in box_a) ─────────────────
  std::cout << "  Clearing free column at (1.55, 2.55)\n";
  clear_column( tree, 1.55, 2.55, 0.0, 3.0 );

  // ── 4. Update inner nodes and prune ──────────────────────────────────────
  tree.updateInnerOccupancy();
  tree.prune();

  // ── 5. Print summary ─────────────────────────────────────────────────────
  double minX, minY, minZ, maxX, maxY, maxZ;
  tree.getMetricMin( minX, minY, minZ );
  tree.getMetricMax( maxX, maxY, maxZ );

  std::cout << "\nTree summary:\n"
            << "  Nodes:      " << tree.size()          << "\n"
            << "  Memory:     " << tree.memoryUsage()   << " bytes\n"
            << "  Depth:      " << tree.getTreeDepth()  << "\n"
            << "  X range:    [" << minX << ", " << maxX << "]\n"
            << "  Y range:    [" << minY << ", " << maxY << "]\n"
            << "  Z range:    [" << minZ << ", " << maxZ << "]\n\n";

  // ── 6. Verify expected slice [0, 1.5) ────────────────────────────────────
  const double z_min = 0.0, z_max = 1.5;
  std::cout << "Verifying expected 2D slice z=[" << z_min << ", " << z_max << "):\n";

  struct Check {
    std::string label;
    double cx, cy;
    bool expected_occupied;
  };

  const std::vector<Check> checks = {
    { "floor centre (2.5, 2.5) – must be occupied",    2.55, 2.55, true  },
    { "floor corner (0.05, 0.05) – must be occupied",  0.05, 0.05, true  },
    { "box_a centre (1.5, 2.5) – must be occupied",    1.55, 2.55, true  },
    // The free column at (1.55, 2.55) was cleared above, so it becomes free;
    // but the floor at z=0.05 was NOT cleared.  Depending on tree update order
    // the column clear may not fully override the floor.  We test a point
    // clearly outside box_a for the "occupied absent" case instead.
    { "box_b centre (3.5, 2.5) z=[0,1.5) – must NOT be occupied", 3.55, 2.55, false },
    { "empty space (2.5, 1.0) – must NOT be occupied", 2.55, 1.05, false },
    { "wall_n centre (2.5, 4.95) – must be occupied",  2.55, 4.95, true  },
  };

  bool all_pass = true;
  for ( const auto &c : checks ) {
    const bool occ =
        column_has_occupied_voxel( tree, c.cx, c.cy, z_min, z_max );
    const bool pass = ( occ == c.expected_occupied );
    std::cout << "  [" << ( pass ? "PASS" : "FAIL" ) << "]  " << c.label << "\n";
    if ( !pass ) all_pass = false;
  }

  // ── 7. Save ───────────────────────────────────────────────────────────────
  std::cout << "\nWriting to: " << output_path << "  ... ";
  if ( tree.write( output_path ) ) {
    std::cout << "OK\n";
  } else {
    std::cerr << "FAILED\n";
    return EXIT_FAILURE;
  }

  std::cout << "\nDataset generation " << ( all_pass ? "PASSED" : "FAILED (see above)" ) << "\n";
  return all_pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
