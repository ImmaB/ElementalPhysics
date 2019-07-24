

#include <Box2D/Dynamics/Ground.h>
#include <Box2D/Dynamics/b2World.h>

Ground::Ground(b2World& world, const def& gd) :
	m_world(world),
	m_ampTilesTileHasChange(1, 1, amp::getGpuAccelView()),
	m_ampTilesChangedIdxs(16, amp::getGpuAccelView()),
	m_ampTiles(16, amp::getGpuAccelView()),
	m_ampMaterials(8, amp::getGpuAccelView())
{
	m_ampMaterials = ampArray<Material>(16, amp::getGpuAccelView());
	m_stride = gd.stride;
	m_sizeY = gd.ySize;
	m_sizeX = gd.xSize;
	m_size = m_sizeY * m_sizeX;
	amp::resize(m_ampTiles, m_size);

	amp::resize(m_ampTilesTileHasChange, m_sizeY / TILE_SIZE_SQRT + 1);
	amp::resize2ndDim(m_ampTilesTileHasChange, m_sizeX / TILE_SIZE_SQRT + 1);
	amp::fill(m_ampTilesTileHasChange, 0);
	amp::resize(m_ampTilesChangedIdxs, m_size);
}

void Ground::SetTiles(Tile* tiles)
{
	m_tiles = vector(tiles, tiles + m_size);
	amp::copy(m_tiles, m_ampTiles, m_size);
}

int32 Ground::CreateMaterial(Material::def md)
{
	int32 idx = 0;
	for (idx = 0; idx < m_materials.size(); idx++)
		if (m_materials[idx].compare(md)) return idx;

	m_materials.push_back(Material(md));
	amp::resize(m_ampMaterials, m_materials.size());
	amp::copy(m_materials, m_ampMaterials, idx);
	m_allMaterialFlags |= md.flags;
	return idx;
}

void Ground::CopyChangedTiles()
{
	if (!m_world.GetParticleSystem()->m_accelerate) return;

	/*std::async(launch::async, [=]()
	{
		ampArrayView<int32> ampChangedCnt(1);
		amp::fill(ampChangedCnt, 0);
		auto& ampTiles = m_ampTiles;
		auto& tilesTileHasChange = m_ampTilesTileHasChange;
		auto& ampChangedIdxs = m_ampTilesChangedIdxs;
		const int32 xSize = m_sizeX;
		
		amp::forEach2DTiled<TILE_SIZE_SQRT, TILE_SIZE_SQRT>(m_sizeY, m_sizeX, [=, &ampTiles,
			&tilesTileHasChange, &ampChangedIdxs](const ampTiledIdx2D<TILE_SIZE_SQRT, TILE_SIZE_SQRT> tIdx) restrict(amp)
		{
			if (!tilesTileHasChange[tIdx.tile]) return;
			const int32 gtIdx = tIdx.global[0] * xSize + tIdx.global[1];
			if (!ampTiles[gtIdx].getChanged()) return;
			ampChangedIdxs[amp::atomicAdd(ampChangedCnt[0], 1)] = gtIdx;
		});
		amp::fill(tilesTileHasChange, 0);
		int32 changedCnt = ampChangedCnt[0];
		if (!changedCnt) return;
		
		m_changedTileIdxs.resize(changedCnt);
		amp::copy(ampChangedIdxs, m_changedTileIdxs, changedCnt);
		
		amp::copy(ampTiles, m_tiles);
		
		std::vector<Tile> changedTiles;
		changedTiles.resize(changedCnt);
		for (int32 i = 0; i < changedCnt; i++)
			changedTiles[i] = m_tiles[m_changedTileIdxs[i]];
		
		if (m_changeCallback)
			m_changeCallback(m_changedTileIdxs.data(), changedTiles.data(), changedCnt);

	});*/

	ampArrayView<int32> hasChange(1);
	amp::fill(hasChange, 0);
	auto& tilesTileHasChange = m_ampTilesTileHasChange;
	amp::forEach2D<TILE_SIZE_SQRT, TILE_SIZE_SQRT>(m_sizeY / TILE_SIZE_SQRT + 1,
		m_sizeX / TILE_SIZE_SQRT + 1, [=, &tilesTileHasChange](int y, int x) restrict(amp)
		{
			if (int32& tileHasChange = tilesTileHasChange[y][x]; tileHasChange)
			{
				tileHasChange = 0;
				hasChange[0] = 1;
			}
		});
	if (!hasChange[0]) return;
	amp::copy(m_ampTiles, m_tiles);
	if (m_changeCallback)
		m_changeCallback(nullptr, m_tiles.data(), 0);
}
