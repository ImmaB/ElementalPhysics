
#pragma once

#include <Box2D/Common/b2Math.h>
#include <Box2D/Amp/ampAlgorithms.h>

using namespace std;

class b2World;

class Ground
{
public:
	struct Tile
	{
		enum Flags
		{
			changed = 1 << 0,
			wet = 1 << 1,
		};
		int32 matIdx;
		float32 height;
		int32 particleCnt;
		int32 particleIdxs[8];
		int32 textureSeed;
		uint32 flags;

		inline bool getChanged() { if (flags & Flags::changed) { flags &= ~Flags::changed; return true; } return false; }
		inline bool getChanged() restrict(amp) { if (flags & Flags::changed) { flags &= ~Flags::changed; return true; } return false; }
		inline void setChanged() { flags |= Flags::changed; }
		inline void setChanged() restrict(amp) { flags |= Flags::changed; }
		inline bool isWet() const { return flags & Flags::wet; }
		inline bool isWet() const restrict(amp) { return flags & Flags::wet; }
		inline bool setWet() { flags |= Flags::wet; }
		inline bool atomicAddFlag(uint32 flag) restrict(amp)
		{
			if (amp::atomicAddFlag(flags, flag)) { setChanged(); return true; }
			return false;
		}
		inline void remWet() { flags &= ~Flags::wet; setChanged(); }
		inline void remWet() restrict(amp) { flags &= ~Flags::wet; setChanged(); }
	};
	typedef void(__stdcall* ChangeCallback)(int32*, Ground::Tile*, int32);

	struct Mat
	{
		enum Flags
		{
			waterRepellent = 1 << 0,
		};

		struct def
		{
			def() : friction(0.0f), bounciness(0.0f), flags(0) {}

			float32 friction;
			float32 bounciness;
			int32 particleCapacity;
			uint32 flags;
		};

		Mat(const def& d) :
			friction(d.friction),
			bounciness(d.bounciness),
			particleCapacity(d.particleCapacity),
			flags(d.flags)
		{}

		float32 friction;
		float32 bounciness;
		int32 particleCapacity;
		uint32 flags;

		bool compare(const def& d)
		{
			return friction == d.friction &&
				bounciness == d.bounciness &&
				particleCapacity == d.particleCapacity &&
				flags == d.flags;
		}

		inline bool isWaterRepellent() const { return flags & Flags::waterRepellent; }
		inline bool isWaterRepellent() const restrict(amp) { return flags & Flags::waterRepellent; }
	};


	struct def
	{
		int32 xSize;
		int32 ySize;
		float32 stride;
	};

	b2World& m_world;

	ChangeCallback m_changeCallback;
	future<void> m_futureChange;
	vector<int32> m_changedTileIdxs;
	vector<Tile> m_changedTiles;

	float32 m_stride;
	float32 m_invStride;
	int32 m_tileCntY, m_tileCntX, m_tileCnt;
	b2Vec2 m_size;
	int32 m_chunkCntY;
	int32 m_chunkCntX;
	int32 m_chunkCnt;
	vector<Tile> m_tiles;
	ampArray<Tile> m_ampTiles;
	ampArray<int32> m_ampChunkHasChange;
	ampArray<int32> m_ampTilesChangedIdxs;

	vector<Mat> m_materials;
	ampArray<Mat> m_ampMaterials;
	int32 m_allMaterialFlags;

	Ground(b2World& world, const def& gd);

	void SetTiles(Tile* tiles);

	int32 CreateMaterial(Mat::def gmd);

	void CopyChangedTiles();

	Tile GetTileAt(const b2Vec2& p) const;
	Ground::Mat GetMat(const Ground::Tile& tile);


private:

	int32 GetIdx(const b2Vec2& p) const;

};

