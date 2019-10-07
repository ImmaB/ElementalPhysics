

#include <Box2D/Dynamics/Ground.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

Ground::Ground(b2World& world, const def& gd) :
	m_world(world),
	m_ampChunkHasChange(1, amp::getGpuAccelView()),
	m_ampTilesChangedIdxs(16, amp::getGpuAccelView()),
	m_ampTiles(16, amp::getGpuAccelView()),
	m_ampMaterials(8, amp::getGpuAccelView())
{
	m_ampMaterials = ampArray<Mat>(16, amp::getGpuAccelView());
	m_stride = gd.stride;
	m_invStride = 1 / gd.stride;
	m_halfStride = gd.stride / 2;
	m_tileCntY = gd.ySize;
	m_tileCntX = gd.xSize;
	m_tileCnt = m_tileCntY * m_tileCntX;
	m_size = Vec2(m_tileCntX, m_tileCntY) * m_stride;
	m_chunkCntY = m_tileCntY / TILE_SIZE_SQRT + 1;
	m_chunkCntX = m_tileCntX / TILE_SIZE_SQRT + 1;
	m_chunkCnt = m_chunkCntY * m_chunkCntX;
	amp::resize(m_ampTiles, m_tileCnt);

	amp::resize(m_ampChunkHasChange, m_chunkCnt);
	amp::fill(m_ampChunkHasChange, 0);
}
Ground::~Ground()
{
	m_materials.clear();
}

void Ground::SetTiles(Tile* tiles)
{
	m_tiles = vector(tiles, tiles + m_tileCnt);
	amp::copy(m_tiles, m_ampTiles, m_tileCnt);
}

int32 Ground::CreateMaterial(Mat::def md)
{
	int32 idx = 0;
	for (idx = 0; idx < m_materials.size(); idx++)
		if (m_materials[idx].compare(md)) return idx;

	m_materials.push_back(Mat(md));
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
	if (!HasChange()) return;
	m_world.d11Device.copy(m_ampTiles, m_d11Tiles, m_tileCnt);
	m_tileCopyFuture = amp::copyAsync(m_ampTiles, m_tiles);
	if (m_changeCallback)
		m_changeCallback(nullptr, m_tiles.data(), m_tileCnt);
	m_hasChange = false;
}


Ground::Tile Ground::GetTileAt(const Vec2& p) const
{
	if (!IsPositionInGrid(p)) return Tile();
	return m_tiles[GetIdx(p)];
}

Ground::Mat Ground::GetMat(const Ground::Tile& tile)
{
	return m_materials[tile.matIdx];
}

void Ground::ExtractParticles(const b2Shape& shape, const b2Transform& transform,
							  int32 partMatIdx, uint32 partFlags, float32 probability)
{
	vector<Vec3> positions;
	const float32 heightOffset = m_world.GetParticleSystem()->GetRadius() + b2_linearSlop;
	auto range = ForEachTileInsideShape(shape, transform, [=, &positions](Tile& tile, const Vec2& tileCenter)
	{
		if (!tile.particleCnt) return;
		for (int32 i = tile.particleCnt - 1; i >= 0; i--)
		{
			if (tile.particleMatIdxs[i] != partMatIdx) continue;
			if (Random() > probability) return;
			tile.removeParticle(i);
			const Vec3 p(GetRandomTilePosition(tileCenter), tile.height + heightOffset);
			positions.push_back(p);
			return;
		}
	});
	range.second = b2Max(range.second, m_tileCnt);
	if (positions.empty()) return;
	ParticleGroup::Def pgd;
	pgd.particleCount = positions.size();
	pgd.positions = positions;
	pgd.matIdx = partMatIdx;
	pgd.flags = partFlags;
	pgd.heat = m_world.m_roomTemperature;
	auto copyFuture = amp::copyAsync(m_tiles, m_ampTiles, range.first, range.second - range.first);
	m_world.GetParticleSystem()->CreateGroup(pgd);
	copyFuture.wait();
	m_hasChange = true;
}

template<typename F>
pair<int32, int32> Ground::ForEachTileInsideShape(const b2Shape& shape, const b2Transform& transform, F& function)
{
	if (shape.m_type == b2Shape::Type::e_chain || shape.m_type == b2Shape::Type::e_edge) return pair<int32, int32>(0, 0);
	b2AABB b;
	shape.ComputeAABB(b, transform, 0);
	int32 lowerXIdx = GetIdx(b.lowerBound.x), upperXIdx = GetIdx(b.upperBound.x),
		  lowerYIdx = GetIdx(b.lowerBound.y), upperYIdx = GetIdx(b.upperBound.y);
	m_tileCopyFuture.wait();
	for (int32 y = lowerYIdx; y <= upperYIdx; y++) for (int32 x = lowerXIdx; x <= upperXIdx; x++)
	{
		const Vec2 p = GetTileCenter(x, y);
		if (IsPositionInGrid(p) && shape.TestPoint(transform, Vec3(p, 0)))
			function(m_tiles[GetIdx(x, y)], p);
	}
	return pair<int32, int32>(GetIdx(lowerXIdx, lowerYIdx), GetIdx(upperXIdx, upperYIdx));
}

bool Ground::HasChange()
{
	if (m_hasChange) return true;

	ampArrayView<int32> hasChange(1);
	amp::fill(hasChange, 0);
	auto& chunkHasChanged = m_ampChunkHasChange;
	amp::forEach(m_chunkCnt, [=, &chunkHasChanged](int32 i) restrict(amp)
	{
		if (int32& chunkHasChange = chunkHasChanged[i]; chunkHasChange)
		{
			chunkHasChange = 0;
			hasChange[0] = 1;
		}
	});
	return hasChange[0];
}
