#pragma once

#include <amp.h>
#include <Box2D/Common/b2Math.h>


template <int N>
inline int32 getNextMultiple(const int32 value)
{
	return value % N ? value + (N - value % N) : value;
}
template <int TileSize>
inline int32 getTileCnt(const int32 value)
{
	return value % TileSize ? value / TileSize + 1 : value / TileSize;
}

namespace amp
{
	static ampExtent getTilableExtent(const int32 size)
	{
		return ampExtent(((size + TILE_SIZE - 1) / TILE_SIZE) * TILE_SIZE);
	}
	static ampExtent getTilableExtent(const int32 size, int32 & tileCnt)
	{
		tileCnt = ((size + TILE_SIZE - 1) / TILE_SIZE);
		return ampExtent(tileCnt * TILE_SIZE);
	}

	template<typename T>
	static void copy(const ampArray<T>& a, std::vector<T>& v, const int32 size = 0)
	{
		if (size)
			Concurrency::copy(a.section(0, size), v.data());
		else
			Concurrency::copy(a, v.data());
	}
	template<typename T>
	static void copy(const std::vector<T>& vec, ampArray<T>& a, const int32 size = 0)
	{
		if (size)
			Concurrency::copy(vec.data(), vec.data() + size, a);
		else
			Concurrency::copy(vec.data(), vec.data() + vec.size(), a);
	}
	template<typename T>
	static void copy(const std::vector<T>& vec, ampArray<T>& a, const int32 start, const int32 size)
	{
		Concurrency::copy(vec.data() + start, vec.data() + start + size, a.section(start, size));
	}
	template<typename T>
	static void copy(const ampArray<T>& a, const int32 idx, T& dest)
	{
		Concurrency::copy(a.section(idx, 1), &dest);
	}
	template<typename T>
	static void copy(const ampArrayView<T>& a, const int32 idx, T& dest)
	{
		Concurrency::copy(a.section(idx, 1), &dest);
	}
	template<typename T>
	static void copy(const T& elem, ampArray<T>& dest, const int32 idx)
	{
		Concurrency::copy(&elem, &elem + 1, dest.section(idx, 1));
	}

	template<typename T>
	static ampCopyFuture copyAsync(const ampArray<T>& a, std::vector<T>& v, const int32 size = 0)
	{
		if (size)
			return Concurrency::copy_async(a.section(0, size), v.data());
		else
			return Concurrency::copy_async(a, v.data());
	}
	template<typename T>
	static ampCopyFuture copyAsync(const std::vector<T>& vec, ampArray<T>& a, const int32 size = 0)
	{
		if (size)
			return Concurrency::copy_async(vec.data(), vec.data() + size, a);
		else
			return Concurrency::copy_async(vec.data(), vec.data() + vec.size(), a);
	}
	template<typename T>
	static ampCopyFuture copyAsync(const ampArray<T>& a, const int32 idx, T& dest)
	{
		return Concurrency::copy_async(a.section(idx, 1), &dest);
	}
	template<typename T>
	static ampCopyFuture copyAsync(const ampArrayView<T>& a, const int32 idx, T& dest)
	{
		return Concurrency::copy_async(a.section(idx, 1), &dest);
	}


	template <typename T>
	static void fill(ampArray<T>& a, const T& value, const int32 size = 0)
	{
		const ampExtent e = size ? ampExtent(size) : a.extent;
		Concurrency::parallel_for_each(e, [=, &a](ampIdx idx) restrict(amp)
		{
			a[idx] = value;
		});
	}
	template <typename T>
	static void fill(ampArrayView<T>& av, const T& value)
	{
		Concurrency::parallel_for_each(av.extent, [=](ampIdx idx) restrict(amp)
		{
			av[idx] = value;
		});
	}
	
	template <typename T>
	static void resize(ampArray<T>& a, const int32 size, const int32 copyCnt = 0)
	{
		if (copyCnt)
		{
			ampArray<T> newA(size, a.accelerator_view);
			if (copyCnt == a.extent[0])
				copy(a, newA);
			else
				a.section(0, copyCnt).copy_to(newA.section(0, copyCnt));
			a = newA;
		}
		else
			a = ampArray<T>(size, a.accelerator_view);
	}
	template <typename T>
	static void resizeStaging(ampArray<T>& a, const int32 size)
	{
		a = ampArray<T>(size, a.accelerator_view, a.associated_accelerator_view);
	}

	template <typename F>
	static void forEach(const int32 cnt, const F& function)
	{
		if (!cnt) return;
		Concurrency::parallel_for_each(ampExtent(cnt).tile<TILE_SIZE>().pad(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			if (const int32 i = tIdx.global[0]; i < cnt)
				function(i);
		});
	}
	template <int T1, int T2, typename F>
	static void forEach2D(const int32 cnt1, const int32 cnt2, const F& function)
	{
		if (!cnt1 || !cnt2) return;
		Concurrency::parallel_for_each(ampExtent2D(cnt1, cnt2).tile<T1, T2>().pad(), [=](ampTiledIdx2D<T1, T2> tIdx) restrict(amp)
		{
			const ampIdx2D idx = tIdx.global;
			const int32 i1 = idx[0];
			const int32 i2 = idx[1];
			if (const int32 i1 = tIdx.global[0], i2 = tIdx.global[0]; i1 < cnt1 && i2 < cnt2)
				function(i1, i2);
		});
	}
	template<typename F>
	static void forEach(const int32 start, const int32 end, const F& function)
	{
		if (const int32 cnt = end - start; cnt > 0)
		{
			Concurrency::parallel_for_each(ampExtent(cnt).tile<TILE_SIZE>().pad(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
			{
				const int32 i = tIdx.global[0] + start;
				if (i < end)
					function(i);
			});
		}
	}
	template <typename F>
	static void forEachTiled(const int32 elemCnt, F& function)
	{
		if (!elemCnt) return;
		Concurrency::parallel_for_each(ampExtent(elemCnt).tile<TILE_SIZE>().pad(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			if (tIdx.local[0] < elemCnt)
				function(tIdx.global[0], tIdx.tile[0], tIdx.local[0]);
		});
	}
	template <typename F>
	static void forEachTiledWithBarrier(const int32 elemCnt, F& function)
	{
		if (!elemCnt) return;
		Concurrency::parallel_for_each(ampExtent(elemCnt).tile<TILE_SIZE>().pad(), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			function(tIdx);
		});
	}

	static uint32 reduceFlags(const ampArray<uint32>& a, const int32 elemCnt)
	{
		if (!elemCnt) return 0;
		ampArrayView<uint32> flagBits(32);
		fill(flagBits, 0u);
		Concurrency::parallel_for_each(a.extent.tile<TILE_SIZE>().pad(), [=, &a](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			const int32 gi = tIdx.global[0];
			const int32 li = tIdx.local[0];
			tile_static int32 lFlagBits[32];
			if (li < 32)
				lFlagBits[li] = 0;
			tIdx.barrier.wait_with_tile_static_memory_fence();
			if (gi < elemCnt)
				for (int32 i = 0; i < 32; i++)
					if (a[gi] & (1 << i))
						lFlagBits[i] = 1;
			tIdx.barrier.wait_with_tile_static_memory_fence();
			if (li == 0)
				for (uint32 i = 0; i < 32; i++)
					if (lFlagBits[i])
						flagBits[i] = 1;
		});
		uint32 particleFlags = 0;
		for (int32 i = 0; i < 32; i++)
			if (flagBits[i])
				particleFlags |= 1 << i;
		return particleFlags;
	}

	/*template<typename F>
	static uint32 reduceOld(const ampArray<uint32>& cnts, const uint32 size, const F& function)
	{
		const uint32 sizeMax = cnts.extent[0];
		ampArray<uint32> cntSums(cnts, m_gpuAccelView);
		for (uint32 stride = 2, halfStride = 1; stride <= sizeMax; halfStride = stride, stride *= 2)
		{
			Concurrency::parallel_for_each(ampExtent(sizeMax / stride), [=, &cntSums](ampIdx idx) restrict(amp)
			{
				const uint32 i = (idx[0] + 1) * stride - 1;
				cntSums[i] += cntSums[i - halfStride];
			});
		}
		uint32 reducedCnt;
		ampCopyFuture reducedCntPromise = copyAsync(cntSums, sizeMax - 1, reducedCnt);
		forEach(size, [=, &cntSums, &cnts](const int32 i) restrict(amp)
		{
			const uint32 cnt = cnts(i);
			if (!cnt) return;
			uint32 cntSum = cntSums[i];
			for (uint32 halfStride = sizeMax / 2, stride = sizeMax; halfStride > 1; stride = halfStride, halfStride /= 2)
			{
				if ((i % stride) >= halfStride)
				{
					const uint32 strideRelIdx = (i % halfStride) + 1;
					if (strideRelIdx != halfStride)
						cntSum += cntSums[i - strideRelIdx];
				}
			}
			function(i, cntSum - cnt);
		});
		reducedCntPromise.wait();
		return reducedCnt;
	}*/


	namespace _reduce_detail
	{
		template<int TileSize>
		inline int32 scanTileExclusive(int32* const tileData, ampTiledIdx<TileSize> tIdx) restrict(amp)
		{
			const int li = tIdx.local[0];

			for (uint32 stride = 1; stride <= (TileSize / 2); stride *= 2)
			{
				if ((li + 1) % (stride * 2) == 0)
					tileData[li] += tileData[li - stride];
				tIdx.barrier.wait_with_tile_static_memory_fence();
			}

			if (li == 0)
				tileData[TileSize - 1] = 0;
			tIdx.barrier.wait_with_tile_static_memory_fence();

			for (int stride = TileSize / 2; stride >= 1; stride /= 2)
			{
				if ((li + 1) % (stride * 2) == 0)
				{
					const int32 tmp = tileData[li];
					tileData[li] += tileData[li - stride];
					tileData[li - stride] = tmp;
				}
				tIdx.barrier.wait_with_tile_static_memory_fence();
			}
			return tileData[TileSize - 1];
		}
	};

	template<typename F>
	static int32 reduce(const ampArray<int32>& cnts, const int32 elemCnt, const F& function)
	{
		using namespace concurrency;
        const auto computeDomain = ampExtent(elemCnt).tile<TILE_SIZE>().pad();
		const int32 tileCnt = computeDomain[0] / TILE_SIZE;
        ampArray<int32> tileSums(tileCnt);
        ampArrayView<int32> tileSumsView(tileSums);
		ampArrayView<const int32> inputView(cnts);
		ampArrayView<int32> outputView(elemCnt);

        // 1 & 2. Scan all tiles and store results in tile_results.
        parallel_for_each(computeDomain, [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
        {
            const int32 gi = tIdx.global[0];
			const int32 li = tIdx.local[0];
            tile_static int32 tileData[TILE_SIZE];
			const int32 origValue = tileData[li] = (gi < elemCnt) ? inputView[gi] : 0;
            tIdx.barrier.wait_with_tile_static_memory_fence();

            const int32 val = _reduce_detail::scanTileExclusive<TILE_SIZE>(tileData, tIdx);
            if (li == (TILE_SIZE - 1))
                tileSumsView[tIdx.tile] = val + origValue;

			if (gi < elemCnt)
				outputView[gi] = tileData[li];// -inputView[gi];
        });

        // 3. Scan tile results.
        //if (tileCnt > TILE_SIZE)
		//  reduce(tileSums, tileCnt, [=](const int32 i, const int32 wi) restrict(amp)
		//	{
		//		tileSumsView[i] = wi;
		//	});
        //else
        {
            parallel_for_each(computeDomain, [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
            {
                const int32 gi = tIdx.global[0];
                const int32 li = tIdx.local[0];

                tile_static int32 tileData[TILE_SIZE];
				tileData[li] = (gi < tileCnt) ? tileSumsView[gi] : 0;

                tIdx.barrier.wait_with_tile_static_memory_fence();

				_reduce_detail::scanTileExclusive<TILE_SIZE>(tileData, tIdx);

                tileSumsView[gi] = tileData[li];
                tIdx.barrier.wait_with_tile_static_memory_fence();
            });
        }
		int32 lastStart, lastTileStart;
		ampCopyFuture lastTileStartProm = copyAsync(tileSumsView, tileCnt - 1, lastStart);
		ampCopyFuture lastStartProm = copyAsync(outputView, elemCnt - 1, lastTileStart);

        // 4. Add the tile results to the individual results for each tile.

        parallel_for_each(computeDomain, [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
        {
            const int32 gi = tIdx.global[0];
			if (gi < elemCnt)
				function(gi, outputView[gi] + tileSumsView[tIdx.tile]);
        });
		lastTileStartProm.wait();
		lastStartProm.wait();
		return lastStart + lastTileStart;
	}

	namespace _radix_sort_detail
	{
		inline uint32 getBits(uint32 x, uint32 numbits, uint32 bitoffset) restrict(amp)
		{
			return (x >> bitoffset) & ~(~0u << numbits);
		}
		inline uint32 pow2(uint32 x) restrict(amp)
		{
			return 1u << x;
		}
		template<int TileSize>
		inline uint32 tileSum(uint32 x, ampTiledIdx<TileSize>& tidx) restrict(amp)
		{
			uint32 li = tidx.local[0];
			tile_static uint32 lSums[TileSize][2];

			lSums[li][0] = x;
			tidx.barrier.wait_with_tile_static_memory_fence();

			for (uint32 i = 0; i < 8; i++)
			{
				if (li < pow2(7 - i))
				{
					uint32 w = (i + 1) % 2;
					uint32 r = i % 2;

					lSums[li][w] = lSums[li * 2][r] + lSums[li * 2 + 1][r];
				}
				tidx.barrier.wait_with_tile_static_memory_fence();
			}
			return lSums[0][0];
		}
		template<int TileSize>
		inline uint32 tilePrefixSum(uint32 x, ampTiledIdx<TileSize> tidx, uint32& lastVal) restrict(amp)
		{
			const uint32 li = tidx.local[0];
			tile_static uint32 lPrefixSums[TileSize][2];

			lPrefixSums[li][0] = x;
			tidx.barrier.wait_with_tile_static_memory_fence();

			for (uint32 i = 0; i < 8; i++)
			{
				const uint32 pow2i = pow2(i);

				const uint32 w = (i + 1) % 2;
				const uint32 r = i % 2;

				lPrefixSums[li][w] = (li >= pow2i) ? (lPrefixSums[li][r] + lPrefixSums[li - pow2i][r]) : lPrefixSums[li][r];

				tidx.barrier.wait_with_tile_static_memory_fence();
			}
			lastVal = lPrefixSums[TileSize-1][0];

			return (li == 0) ? 0 : lPrefixSums[li - 1][0];
		}
		template<int TileSize>
		inline uint32 tilePrefixSum(uint32 x, ampTiledIdx<TileSize> tIdx) restrict(amp)
		{
			uint32 ll = 0;
			return tilePrefixSum(x, tIdx, ll);
		}
		template<int TileSize>
		static void calcIntermSums(const uint32 bitoffset, ampArrayView<Proxy>& intermArr,
			ampArray<uint32>& intermSums, ampArray<uint32>& intermPrefixSums)
		{
			const uint32 QuarterTile = TileSize / 4;
			const auto computeDomain = intermArr.extent.tile<TileSize>().pad();
			const uint32 tileCnt = computeDomain.size() / TileSize;
			Concurrency::parallel_for_each(computeDomain, [=, &intermSums](ampTiledIdx<TileSize> tIdx) restrict(amp)
			{
				const bool inbound = (tIdx.global[0] < intermArr.extent[0]);
				uint32 num = (inbound) ? getBits(intermArr[tIdx.global[0]].tag, 2, bitoffset) : getBits(0xffffffff, 2, bitoffset);
				for (uint32 i = 0; i < 4; i++)
				{
					const uint32 sum = tileSum<TileSize>(num == i, tIdx);
					if (tIdx.local[0] == 0)
						intermSums[i * tileCnt + tIdx.tile[0]] = sum;
				}
			});

			const uint32 tileCnt4 = tileCnt * 4;
			const uint32 numiter = (tileCnt / QuarterTile) + ((tileCnt % QuarterTile == 0) ? 0 : 1);
			Concurrency::parallel_for_each(ampExtent(TileSize).tile<TileSize>(), [=, &intermPrefixSums, &intermSums](ampTiledIdx<TileSize> tIdx) restrict(amp)
			{
				uint32 lastVal0 = 0;
				uint32 lastVal1 = 0;

				for (uint32 i = 0; i < numiter; i++)
				{
					const uint32 gi = tIdx.local[0] + i * TileSize;
					uint32 num = (gi < tileCnt4) ? intermSums[gi] : 0;
					const uint32 scan = tilePrefixSum(num, tIdx, lastVal0);
					if (gi < tileCnt4) intermPrefixSums[gi] = scan + lastVal1;

					lastVal1 += lastVal0;
				}
			});
		}
		template<int TileSize>
		static void radixSortStep(const uint32 bitoffset, const ampArrayView<const Proxy>& src, ampArrayView<Proxy>& dest,
			ampArray<uint32>& intermPrefixSums)
		{
			const auto computeDomain = src.extent.tile<TileSize>().pad();
			const uint32 tileCnt = computeDomain.size() / TileSize;
			Concurrency::parallel_for_each(computeDomain, [=, &intermPrefixSums](ampTiledIdx<TileSize> tidx) restrict(amp)
			{
				const int32 gi = tidx.global[0];
				const bool inbounds = (gi < src.extent[0]);
				const Proxy element = inbounds ? src[gi] : Proxy(0, 0xffffffff);
				uint32 num = getBits(element.tag, 2, bitoffset);
				for (uint32 i = 0; i < 4; i++)
				{
					const uint32 scan = tilePrefixSum<TileSize>((num == i), tidx) + intermPrefixSums[i * tileCnt + tidx.tile[0]];
					if (num == i && inbounds) dest[scan] = element;
				}
			});
		}
	}
	static void radixSort(ampArray<Proxy>& a, const uint32 size)
	{
		const int32 tileCnt = getTileCnt<TILE_SIZE>(size);
		ampArray<Proxy> intermArr(size);
		ampArray<uint32> intermSums(tileCnt * 4);
		ampArray<uint32> intermPrefixSums(tileCnt * 4);

		ampArrayView<Proxy> av = a.section(0, size);
		for (uint32 i = 0; i < 16; i++)
		{
			ampArrayView<Proxy>& src = (i % 2 == 0) ? av : intermArr;
			ampArrayView<Proxy>& dest = (i % 2 == 0) ? intermArr : av;

			const uint32 bitoffset = i * 2;
			_radix_sort_detail::calcIntermSums<TILE_SIZE>(bitoffset, src, intermSums, intermPrefixSums);
			_radix_sort_detail::radixSortStep<TILE_SIZE>(bitoffset, src, dest, intermPrefixSums);
		}
	}

	inline void atomicAdd(float32& dest, const float32 add) restrict(amp)
	{
		float32 expected = dest;
		float32 newValue = expected + add;
		while (!Concurrency::atomic_compare_exchange((uint32*)& dest, (uint32*)& expected, reinterpret_cast<uint32&>(newValue)))
			newValue = expected + add;
	}
	inline void atomicSub(float32& dest, const float32 sub) restrict(amp)
	{
		float32 expected = dest;
		float32 newValue = expected - sub;
		while (!Concurrency::atomic_compare_exchange((uint32*)& dest, (uint32*)& expected, reinterpret_cast<uint32&>(newValue)))
			newValue = expected - sub;
	}
	inline void atomicAdd(b2Vec2& dest, const b2Vec2& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
	}
	inline void atomicSub(b2Vec2& dest, const b2Vec2& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
	}
	inline void atomicAdd(b2Vec3& dest, const b2Vec2& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
	}
	inline void atomicSub(b2Vec3& dest, const b2Vec2& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
	}
	inline void atomicAdd(b2Vec3& dest, const b2Vec3& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
		atomicAdd(dest.z, add.z);
	}
	inline void atomicSub(b2Vec3& dest, const b2Vec3& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
		atomicSub(dest.z, sub.z);
	}
};


/*
namespace _radix_sort_details
	{


		//----------------------------------------------------------------------------
		// Byte pack and unpack
		//----------------------------------------------------------------------------

		template<int index, typename T>
		inline uint32 pack_byte(const T& value) restrict(cpu, amp)
		{
			static_assert(index < sizeof(T), "Index out of range.");
			return (static_cast<uint32>(value) && 0xFF) << (index * CHAR_BIT);
		}

		template<typename T>
		inline uint32 pack_byte(const T& value, const unsigned index) restrict(cpu, amp)
		{
			return (static_cast<uint32>(value) && 0xFF) << (index * CHAR_BIT);
		}

		template<int index, typename T>
		inline unsigned int unpack_byte(const T& value) restrict(cpu, amp)
		{
			static_assert(index < sizeof(T), "Index out of range.");
			return (value >> (index * CHAR_BIT)) & 0xFF;
		}

		template<typename T>
		inline unsigned int unpack_byte(const T & value, const unsigned index) restrict(cpu, amp)
		{
			return (value >> (index * CHAR_BIT)) & 0xFF;
		}

		template<typename T>
		unsigned int bit_count() restrict(cpu, amp)
		{
			return sizeof(T)* CHAR_BIT;
		}

		//----------------------------------------------------------------------------
		// container padded_read & padded_write
		//----------------------------------------------------------------------------

		template <typename T>
		inline void padded_write(ampArrayView<T> & av, const int gidx, const T & value) restrict(amp)
		{
			if (gidx < av.extent[0])
			{
				av[gidx] = value;
			}
		}

		// TODO: Should this return an extent? Better name. o we even need a specific function for this or can we use extent.contains() instead?
		template <int N, typename InputIndexableView>
		inline int tile_partial_data_size(const InputIndexableView & arr, ampTiledIdx<N> tidx) restrict(amp)
		{
			return arr.extent.size() - tidx.tile[0] * tidx.tile_extent[0];
		}

		template <int TileSize>
		inline int last_index_in_tile(const int tidx, const int extent) restrict(amp)
		{
			return b2Min(((tidx + 1) * TileSize) - 1, extent - 1);
		}


		template <int TileSize, typename T>
		static inline T scan_tile_exclusive(T * const tile_data, ampTiledIdx<TileSize> tidx) restrict(amp)
		{
			const int lidx = tidx.local[0];

			for (int stride = 1; stride <= (TileSize / 2); stride *= 2)
			{
				if ((lidx + 1) % (stride * 2) == 0)
				{
					tile_data[lidx] = tile_data[lidx] + tile_data[lidx - stride];
				}
				tidx.barrier.wait_with_tile_static_memory_fence();
			}

			if (lidx == 0)
			{
				tile_data[TileSize - 1] = 0;
			}
			tidx.barrier.wait_with_tile_static_memory_fence();

			for (int stride = TileSize / 2; stride >= 1; stride /= 2)
			{
				if ((lidx + 1) % (stride * 2) == 0)
				{
					auto tmp = tile_data[lidx];
					tile_data[lidx] = tile_data[lidx] + tile_data[lidx - stride];
					tile_data[lidx - stride] = tmp;
				}
				tidx.barrier.wait_with_tile_static_memory_fence();
			}
			return tile_data[TileSize - 1];
		}

		template <int TileSize, typename T>
		void scan_exclusive(const ampAccelView & accl_view, const ampArrayView<T> & input_view, ampArrayView<T> & output_view)
		{
			const auto compute_domain = output_view.extent.tile<TileSize>().pad();
			ampArray<T> tile_sums(compute_domain[0] / TileSize, accl_view);
			ampArrayView<T> tile_sums_vw(tile_sums);

			// 1 & 2. Scan all tiles and store results in tile_results.

			Concurrency::parallel_for_each(accl_view, compute_domain, [=](ampTiledIdx<TileSize> tidx) restrict(amp)
			{
				const int gi = tidx.global[0];
				const int li = tidx.local[0];
				const int partial_data_length = tile_partial_data_size(output_view, tidx);

				tile_static T tile_data[TileSize];
				tile_data[li] = (li >= partial_data_length) ? 0 : input_view[gi];
				const T current_value = tile_data[li];
				tidx.barrier.wait_with_tile_static_memory_fence();

				auto val = scan_tile_exclusive<TileSize>(tile_data, tidx);

				// This does not execute correctly on Warp accelerators for some reason. Steps Warp A & B do this instead.
				if (li == (TileSize - 1))
					tile_sums_vw[tidx.tile[0]] = val + current_value;

				if (gi < output_view.extent[0])
					output_view[gi] = tile_data[li];

				// padded_write(output_view, gi, tile_data[li]);
			});

			// 3. Scan tile results.

			if (tile_sums_vw.extent[0] > TileSize)
			{
				scan_exclusive<TileSize>(accl_view, tile_sums_vw, tile_sums_vw);
			}
			else
			{
				Concurrency::parallel_for_each(accl_view, compute_domain, [=](ampTiledIdx<TileSize> tIdx) restrict(amp)
					{
						const int gi = tIdx.global[0];
						const int li = tIdx.local[0];
						const int partial_data_length = tile_partial_data_size(tile_sums_vw, tIdx);

						tile_static T tile_data[TileSize];
						tile_data[li] = (li >= partial_data_length) ? 0 : tile_sums_vw[gi];

						tIdx.barrier.wait_with_tile_static_memory_fence();

						scan_tile_exclusive<TileSize>(tile_data, tIdx);

						tile_sums_vw[gi] = tile_data[li];
						tIdx.barrier.wait_with_tile_static_memory_fence();
					});
			}

			// 4. Add the tile results to the individual results for each tile.

			Concurrency::parallel_for_each(accl_view, compute_domain, [=](ampTiledIdx<TileSize> tidx) restrict(amp)
				{
					const int gidx = tidx.global[0];

					if (gidx < output_view.extent[0])
					{
						output_view[gidx] += tile_sums_vw[tidx.tile[0]];
					}
				});
		}

		inline int radix_key_value(const int value, const unsigned key_idx) restrict(amp)
		{
			return (value >> (key_idx * 2)) & 3;
		}

		inline int scan_tile_exclusive(int* const tile_data, ampTiledIdx<TILE_SIZE_HALF> tidx) restrict(amp)
		{
			const int li = tidx.local[0];

			for (int stride = 1; stride <= (TILE_SIZE_HALF / 2); stride *= 2)
			{
				if ((li + 1) % (stride * 2) == 0)
					tile_data[li] = tile_data[li] + tile_data[li - stride];
				tidx.barrier.wait_with_tile_static_memory_fence();
			}

			if (li == 0)
				tile_data[TILE_SIZE_HALF - 1] = 0;
			tidx.barrier.wait_with_tile_static_memory_fence();

			for (int stride = TILE_SIZE_HALF / 2; stride >= 1; stride /= 2)
			{
				if ((li + 1) % (stride * 2) == 0)
				{
					auto tmp = tile_data[li];
					tile_data[li] = tile_data[li] + tile_data[li - stride];
					tile_data[li - stride] = tmp;
				}
				tidx.barrier.wait_with_tile_static_memory_fence();
			}
			return tile_data[TILE_SIZE_HALF - 1];
		}

		template <int tile_size>
		void radix_sort_tile_by_key(Proxy * const tile_data, const int data_size, ampTiledIdx<tile_size> tIdx, const int key_idx) restrict(amp)
		{
			const unsigned bin_count = 1 << 2;
			const int gi = tIdx.global[0];
			const int ti = tIdx.tile[0];
			const int li = tIdx.local[0];

			// Increment histogram bins for each element.

			tile_static uint32 tile_radix_values[tile_size];
			tile_radix_values[li] = pack_byte(1, radix_key_value(tile_data[li].tag, key_idx));
			tIdx.barrier.wait_with_tile_static_memory_fence();

			tile_static uint32 histogram_bins_scan[bin_count];
			if (li == 0)
			{
				// Calculate histogram of radix values. Don't add values that are off the end of the data.
				uint32 global_histogram = 0;
				const int tile_data_size = b2Min(tile_size, (data_size - (ti * tile_size)));
				for (int i = 0; i < tile_data_size; ++i)
				{
					global_histogram += tile_radix_values[i];
				}

				// Scan to get offsets for each histogram bin.

				histogram_bins_scan[0] = 0;
				for (int i = 1; i < bin_count; ++i)
				{
					histogram_bins_scan[i] = unpack_byte(global_histogram, i - 1) + histogram_bins_scan[i - 1];
				}
			}
			tIdx.barrier.wait_with_tile_static_memory_fence();

			scan_tile_exclusive<tile_size>(tile_radix_values, tIdx);

			// Shuffle data into sorted order.

			const Proxy& tmp = tile_data[li];
			tIdx.barrier.wait_with_tile_static_memory_fence();
			if (gi < data_size)
			{
				const int rdx = radix_key_value(tmp.tag, key_idx);
				uint32 dest_idx = histogram_bins_scan[rdx] + unpack_byte(tile_radix_values[li], rdx);
				tile_data[dest_idx] = tmp;
			}
		}
		inline void initialize_bins(int* const bin_data) restrict(amp)
		{
			for (int b = 0; b < 4; ++b)
				bin_data[b] = 0;
		}

		template <typename T>
		inline T segment_exclusive_scan(const ampArrayView<T> & exclusive_scan, const int segment_width, const int i) restrict(amp, cpu)
		{
			return exclusive_scan[i] - exclusive_scan[i - (i % segment_width)];
		}

		static void sortByKey(const ampAccelView& accl_view, const ampArrayView<Proxy>& input_view, ampArrayView<Proxy>& output_view, const int key_idx)
		{
			const ampTiledExt<TILE_SIZE_HALF> compute_domain = output_view.get_extent().tile<TILE_SIZE_HALF>().pad();
			const int tile_count = b2Max(1u, compute_domain.size() / TILE_SIZE_HALF);

			ampArray2D<int> per_tile_rdx_offsets(ampExtent2D(tile_count, 4), accl_view);
			ampArray<int> global_rdx_offsets(4, accl_view);
			ampArray<int> tile_histograms(ampExtent(4 * tile_count), accl_view);

			fill(global_rdx_offsets, 0, 4);

			Concurrency::parallel_for_each(accl_view, compute_domain, [=, &per_tile_rdx_offsets, &global_rdx_offsets, &tile_histograms](ampTiledIdx<TILE_SIZE_HALF> tIdx) restrict(amp)
			{
				const int gi = tIdx.global[0];
				const int ti = tIdx.tile[0];
				const int li = tIdx.local[0];
				tile_static Proxy tile_data[TILE_SIZE_HALF];
				tile_static int per_thread_rdx_histograms[TILE_SIZE_HALF][4];

				// Initialize histogram bins and copy data into tiles.
				initialize_bins(per_thread_rdx_histograms[li]);
				tile_data[li] = gi < input_view.extent[0] ? input_view[gi] : Proxy();

				// Increment radix bins for each element on each tile.
				if (gi < input_view.extent[0])
					per_thread_rdx_histograms[li][radix_key_value(tile_data[li].tag, key_idx)]++;
				tIdx.barrier.wait_with_tile_static_memory_fence();

				// First 4 threads per tile collapse thread values to create the tile histogram.
				if (li < 4)
					for (int i = 1; i < TILE_SIZE_HALF; ++i)
						per_thread_rdx_histograms[0][li] += per_thread_rdx_histograms[i][li];
				tIdx.barrier.wait_with_tile_static_memory_fence();

				// First 4 threads per tile increment counts for global histogram and copies tile histograms to global memory.
				if (li < 4)
					Concurrency::atomic_fetch_add(&global_rdx_offsets[li], per_thread_rdx_histograms[0][li]);

				//output_view[gidx] = (li < 4) ? per_thread_rdx_histograms[0][li] : 0;                                            // Dump per-tile histograms, per_tile_rdx_histograms

				// Exclusive scan the tile histogram to calculate the per-tile offsets.
				if (li < 4)
					tile_histograms[(li * tile_count) + ti] = per_thread_rdx_histograms[0][li];

				tIdx.barrier.wait_with_tile_static_memory_fence();
				scan_tile_exclusive(per_thread_rdx_histograms[0], tIdx);

				if (li < 4)
					per_tile_rdx_offsets[ti][li] = per_thread_rdx_histograms[0][li];
			});

			Concurrency::parallel_for_each(accl_view, compute_domain, [=, &global_rdx_offsets, &tile_histograms](ampTiledIdx<TILE_SIZE_HALF> tidx) restrict(amp)
			{
				const int gidx = tidx.global[0];
				const int idx = tidx.local[0];

				// Calculate global radix offsets from the global radix histogram. All tiles do this but only the first one records the result.
				tile_static int scan_data[TILE_SIZE_HALF];
				scan_data[idx] = (idx < 4) ? global_rdx_offsets[idx] : 0;
				tidx.barrier.wait_with_tile_static_memory_fence();

				scan_tile_exclusive(scan_data, tidx);

				if (gidx < 4)
					global_rdx_offsets[gidx] = scan_data[gidx];
			});

			ampArrayView<int> tile_histograms_vw(tile_histograms);
			scan_exclusive<SCAN_TILE_SIZE>(ampAccel::get_auto_selection_view(), tile_histograms_vw, tile_histograms_vw);

			Concurrency::parallel_for_each(accl_view, compute_domain, [=, &per_tile_rdx_offsets, &tile_histograms, &global_rdx_offsets](ampTiledIdx<TILE_SIZE_HALF> tidx) restrict(amp)
			{
				const int gi = tidx.global[0];
				const int ti = tidx.tile[0];
				const int li = tidx.local[0];

				// Sort elements within each tile.
				tile_static Proxy tile_data[TILE_SIZE_HALF];
				tile_data[li] = gi < input_view.extent[0] ? input_view[gi] : Proxy();

				tidx.barrier.wait_with_tile_static_memory_fence();

				const int keys_per_tile = (2 / 2);
				for (int k = key_idx; k < (key_idx + 1); ++k)
				{
					radix_sort_tile_by_key<TILE_SIZE_HALF>(tile_data, input_view.extent[0], tidx, k);
				}
				tidx.barrier.wait_with_tile_static_memory_fence();

				// Move tile sorted elements to global destination.

				const int rdx = radix_key_value(tile_data[li].tag, key_idx);
				const int dest_gi =
					li -
					per_tile_rdx_offsets[ti][rdx] +
					segment_exclusive_scan(tile_histograms_vw, tile_count, (rdx * tile_count) + ti) +
					global_rdx_offsets[rdx];

				if (gi < input_view.extent[0])
				{
					output_view[dest_gi] = tile_data[li];
				}
			});
		}

	}

	static void radix_sort(ampArray<Proxy>& a, const uint32 size)
	{
		ampArray<Proxy> a2(a.extent, a.accelerator_view);
		ampArrayView<Proxy> av1 = a.section(0, size);
		ampArrayView<Proxy> av2 = a2.section(0, size);
		for (int i = 0; i < 16; ++i)
		{
			_radix_sort_details::sortByKey(a.accelerator_view, av1, av2, i);
			std::swap(av2, av1);
		}
	}

*/