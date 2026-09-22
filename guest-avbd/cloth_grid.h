// SPDX-License-Identifier: Apache-2.0 OR MIT
#ifndef CLOTH_GRID_H
#define CLOTH_GRID_H

#include <cstdint>
#include <vector>

// A rectangular cloth panel as an AVBD FEM mesh: membrane (triangle strain)
// constraints with rest material inverses, cotangent-Laplacian dihedral
// bending, and augmented-Lagrangian attachment pins. Constructed to match
// DiffCloth's Triangle::inv_deltaUV, createBendingConstraints, and the
// Simulation.cpp AVBD upload, so the same Slang kernels solve it. Vertex id
// is y-fastest (ix*ny + iy), matching the demo's make_cloth pin order.
struct ClothMesh {
	uint32_t nx = 0, ny = 0;
	float width = 0.0f, height = 0.0f;

	std::vector<float> positions; // 3*nVerts, tight xyz (rest)
	std::vector<float> mass;      // nVerts

	std::vector<uint32_t> triIdx;   // 3 per triangle
	std::vector<float> triInvUV;    // 4 per triangle (row-major 2x2)

	std::vector<uint32_t> bendIdx;      // 4 per bending stencil
	std::vector<float> bendWeight;      // 4 per stencil (cotangent Laplacian)
	std::vector<float> bendNTarget;     // 1 per stencil (rest magnitude)

	std::vector<uint32_t> attachVert;   // pinned vertex ids (top edge)
	std::vector<float> attachFixed;     // 3 per pin (rest position)

	std::vector<uint32_t> faces;    // 3 indices per render triangle (= triIdx)

	uint32_t nVerts() const { return static_cast<uint32_t>(positions.size() / 3); }
	uint32_t nTri() const { return static_cast<uint32_t>(triInvUV.size() / 4); }
	uint32_t nBend() const { return static_cast<uint32_t>(bendNTarget.size()); }
	uint32_t nAttach() const { return static_cast<uint32_t>(attachVert.size()); }
};

// Build an nx-by-ny panel. The panel lies horizontally at world height
// `panelTop` and hangs under -Y gravity from its two pinned far corners.
ClothMesh build_cloth_mesh(uint32_t nx, uint32_t ny, float width, float height,
		float panelTop, float clothMass);

// Concatenate meshes into one, offsetting each vertex block so indices stay
// disjoint. Used to feed several panels through a single partitioning path.
ClothMesh combine_meshes(const std::vector<ClothMesh> &parts);

// Split a mesh into its connected components over the constraint graph
// (triangles, bending, springs). Each component is a fully independent
// sub-island with remapped indices — solvable on its own thread / sandbox
// with no coupling. A single connected panel returns one island.
std::vector<ClothMesh> partition_islands(const ClothMesh &mesh);

// One tile of a domain-decomposed panel: a sub-mesh whose first `nOwned`
// vertices are solved here and whose trailing vertices are ghosts (a
// neighbour tile's boundary, held fixed and refreshed each substep).
// `globalId` maps every local vertex back to the panel it came from.
struct PanelTile {
	ClothMesh mesh;
	uint32_t nOwned = 0;
	std::vector<uint32_t> globalId;
};

// Cut ONE (possibly connected) panel into `nTiles` vertical bands by x, giving
// each tile its owned vertices plus a one-ring ghost halo. Solving the tiles
// with a fixed per-substep ghost exchange is deterministic additive Schwarz,
// so a single connected cloth still spreads across cores.
std::vector<PanelTile> split_connected_panel(const ClothMesh &mesh, uint32_t nTiles);

#endif // CLOTH_GRID_H
