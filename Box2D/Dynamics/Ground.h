
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
		int32 textureSeed;
		uint32 flags;

		bool getChanged() { if (flags & Flags::changed) { flags &= ~Flags::changed; return true; } return false; }
		bool getChanged() restrict(amp) { if (flags & Flags::changed) { flags &= ~Flags::changed; return true; } return false; }
		void setChanged() { flags |= Flags::changed; }
		void setChanged() restrict(amp) { flags |= Flags::changed; }
		bool isWet() const { return flags & Flags::wet; }
		bool isWet() const restrict(amp) { return flags & Flags::wet; }
		bool setWet() { flags |= Flags::wet; }
		bool atomicSetWet() restrict(amp)
		{
			if (amp::atomicAddFlag(flags, Flags::wet)) { setChanged(); return true; }
			return false;
		}
		void remWet() { flags &= ~Flags::wet; setChanged(); }
		void remWet() restrict(amp) { flags &= ~Flags::wet; setChanged(); }
	};
	typedef void(__stdcall* ChangeCallback)(int32*, Ground::Tile*, int32);

	struct Material
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
			uint32 flags;
		};

		Material(const def& d) :
			friction(d.friction),
			bounciness(d.bounciness),
			flags(d.flags)
		{}

		float32 friction;
		float32 bounciness;
		uint32 flags;

		bool compare(const def& d)
		{
			return friction == d.friction &&
				bounciness == d.bounciness &&
				flags == d.flags;
		}

		bool isWaterRepellent() const { return flags & Flags::waterRepellent; }
		bool isWaterRepellent() const restrict(amp) { return flags & Flags::waterRepellent; }
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
	int32 m_sizeX;
	int32 m_sizeY;
	int32 m_size;
	vector<Tile> m_tiles;
	ampArray<Tile> m_ampTiles;
	ampArray2D<int32> m_ampTilesTileHasChange;
	ampArray<int32> m_ampTilesChangedIdxs;

	vector<Material> m_materials;
	ampArray<Material> m_ampMaterials;
	int32 m_allMaterialFlags;

	Ground(b2World& world, const def& gd);

	void SetTiles(Tile* tiles);

	int32 CreateMaterial(Material::def gmd);

	void CopyChangedTiles();
};

