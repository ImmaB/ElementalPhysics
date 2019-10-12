#pragma once

#include <amp.h>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2Timer.h>
#include <Box2D/Common/b2GlobalVariables.h>
#include <d3d11.h>


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
	static ampAccel* _pAmpDevice = nullptr;

	static void CheckCompatibilty()
	{
		auto accelerators = Concurrency::accelerator::get_all();
		if (accelerators.size() == 0)
			throw ERROR;
	}

	class CopyFuture
	{
	private:
		ampCopyFuture m_future;
	public:
		void set(ampCopyFuture future) { m_future = future; }
		void wait() { if (m_future.valid()) m_future.wait(); }
	};

	static ampAccel getGpuAccel() { return ampAccel(ampAccel::default_accelerator); }
	static ampAccelView getCpuAccelView() { return ampAccel(ampAccel::cpu_accelerator).default_view; }
	static ampAccelView getGpuAccelView() { return ampAccel(ampAccel::default_accelerator).default_view; }

	static int32 getTileCount(const int32 size)
	{
		return ((size + TILE_SIZE - 1) / TILE_SIZE);
	}
	static int32 getTilable(const int32 size)
	{
		return getTileCount(size) * TILE_SIZE;
	}
	static int32 getTilable(const int32 size, int32& tileCnt)
	{
		tileCnt = getTileCount(size);
		return tileCnt * TILE_SIZE;
	}
	static ampExtent getTilableExtent(const int32 size)
	{
		return ampExtent(getTilable(size));
	}
	static ampExtent getTilableExtent(const int32 size, int32& tileCnt)
	{
		return ampExtent(getTilable(size, tileCnt));
	}
	template<int TileSize>
	inline ampTiledExt<TileSize> tileAndPad(int32 cnt) { return ampExtent(cnt).tile<TileSize>().pad(); }

	#define COPY_SYNC template<typename T> void
	#define COPY_ASYNC template<typename T> ampCopyFuture

	COPY_SYNC copy(const ampArray<T>& src, std::vector<T>& dst, const int32 size = 0)
	{
		Concurrency::copy(size ? src.section(0, size) : src, dst.data());
	}
	COPY_ASYNC copyAsync(const ampArray<T>& src, std::vector<T>& dst, const int32 size = 0)
	{
		return Concurrency::copy_async(size ? src.section(0, size) : src, dst.data());
	}

	COPY_SYNC copy(const std::vector<T>& src, ampArray<T>& dst, const int32 size = 0)
	{
		Concurrency::copy(src.data(), src.data() + (size ? size : src.size()), dst);
	}
	COPY_ASYNC copyAsync(const std::vector<T>& src, ampArray<T>& dst, const int32 size = 0)
	{
		return Concurrency::copy_async(src.data(), src.data() + (size ? size : src.size()), dst);
	}

	COPY_SYNC copy(const std::vector<T>& src, ampArrayView<T>& dst, const int32 size = 0)
	{
		Concurrency::copy(src.data(), src.data() + (size ? size : src.size()), dst);
	}

	COPY_SYNC copy(const std::vector<T>& src, ampArray<T>& dst, const int32 start, const int32 size)
	{
		Concurrency::copy(src.data() + start, src.data() + start + size, dst.section(start, size));
	}
	COPY_ASYNC copyAsync(const std::vector<T>& src, ampArray<T>& dst, const int32 start, const int32 size)
	{
		return Concurrency::copy_async(src.data() + start, src.data() + start + size, dst.section(start, size));
	}

	COPY_SYNC copy(const T* src, ampArray<T>& dst, const int32 size)
	{
		if (size) Concurrency::copy(src, src + size, dst);
	}

	COPY_SYNC copy(const ampArray<T>& src, const int32 idx, T& dst)
	{
		Concurrency::copy(src.section(idx, 1), &dst);
	}
	COPY_ASYNC copyAsync(const ampArray<T>& src, const int32 idx, T& dst)
	{
		return Concurrency::copy_async(src.section(idx, 1), &dst);
	}

	COPY_SYNC copy(const ampArrayView<T>& src, const int32 idx, T& dst)
	{
		Concurrency::copy(src.section(idx, 1), &dst);
	}
	COPY_ASYNC copyAsync(const ampArrayView<T>& src, const int32 idx, T& dst)
	{
		return Concurrency::copy_async(src.section(idx, 1), &dst);
	}
	COPY_ASYNC copyAsync(const ampArrayView<const T>& src, const int32 idx, T& dst)
	{
		return Concurrency::copy_async(src.section(idx, 1), &dst);
	}

	COPY_SYNC copy(const ampArrayView<T>& src, std::vector<T>& dst)
	{
		Concurrency::copy(src, dst.data());
	}

	COPY_SYNC copy(const ampArrayView<T>& src, T* dst)
	{
		Concurrency::copy(src, dst);
	}

	COPY_SYNC copy(const T& src, ampArray<T>& dst, const int32 idx)
	{
		Concurrency::copy(&src, &src + 1, dst.section(idx, 1));
	}

	COPY_SYNC copy(const ampArray<T>& src, std::vector<T>& dst, const int32 start, const int32 end)
	{
		if (const int32 size = end - start; size > 0)
			Concurrency::copy(src.section(start, size), dst.data() + start);
	}
	COPY_ASYNC copyAsync(const ampArray<T>& a, std::vector<T>& dst, const int32 start, const int32 end)
	{
		const int32 size = end - start;
		return Concurrency::copy_async(a.section(start, size), dst.data() + start);
	}

	COPY_ASYNC copyAsync(const T* src, ampArray<T>& dst, const int32 size)
	{
		return size ? Concurrency::copy_async(src, src + size, dst) : ampCopyFuture();
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
	static void fill(ampArrayView<T>& av, const T& value, const int32 size)
	{
		const ampExtent e = size ? ampExtent(size) : av.extent;
		Concurrency::parallel_for_each(e, [=](ampIdx idx) restrict(amp)
		{
			av[idx] = value;
		});
	}
	template <typename T>
	static void fill(ampArray<T>& a, const T& value, const int32 start, const int32 end)
	{
		const int32 size = end - start;
		if (size <= 0) return;
		Concurrency::parallel_for_each(ampExtent(size), [=, &a](ampIdx idx) restrict(amp)
		{
			a[start + idx] = value;
		});
	}
	template <typename T>
	static void fill(ampArray2D<T>& a, const T& value, int32 sizeY = 0, int32 sizeX = 0)
	{
		const ampExtent2D e(sizeY ? sizeY : a.extent[0], sizeX ? sizeX : a.extent[1]);
		Concurrency::parallel_for_each(e, [=, &a](ampIdx2D idx) restrict(amp)
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
	static void destroy(ampArray<T>& a)
	{
		a = ampArray<T>(1, a.accelerator_view);
		a.~array();
	}
	template <typename T>
	static void destroy(ampArray2D<T>& a)
	{
		a = ampArray2D<T>(1, 1, a.accelerator_view);
		a.~array();
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
	static void resize(ampArray2D<T>& a, const int32 size, const int32 copyCnt = 0)
	{
		if (copyCnt)
		{
			ampArray2D<T> newA(size, a.extent[1], a.accelerator_view);
			if (copyCnt == a.extent[0])
				copy(a, newA);
			else
				a.section(0, 0, copyCnt, a.extent[1]).copy_to(newA.section(0, 0, copyCnt, a.extent[1]));
			a = newA;
		}
		else
			a = ampArray2D<T>(size, a.extent[1], a.accelerator_view);
	}
	template <typename T>
	static void resize2ndDim(ampArray2D<T>& a, const int32 size)
	{
		a = ampArray2D<T>(a.extent[0], size, a.accelerator_view);
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
		Concurrency::parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			if (const int32 i = tIdx.global[0]; i < cnt)
				function(i);
		});
	}
	template <int T1, int T2, typename F>
	static void forEach2D(const int32 cntY, const int32 cntX, const F& function)
	{
		if (!cntY || !cntX) return;
		Concurrency::parallel_for_each(ampExtent2D(cntY, cntX).tile<T1, T2>().pad(), [=](ampTiledIdx2D<T1, T2> tIdx) restrict(amp)
		{
			const ampIdx2D idx = tIdx.global;
			const int32 i1 = idx[0];
			const int32 i2 = idx[1];
			if (const int32 i1 = tIdx.global[0], i2 = tIdx.global[0]; i1 < cntY && i2 < cntX)
				function(i1, i2);
		});
	}
	template <int T1, int T2, typename F>
	static void forEach2DTiled(const int32 cntY, const int32 cntX, const F& function)
	{
		if (!cntY || !cntX) return;
		Concurrency::parallel_for_each(ampExtent2D(cntY, cntX).tile<T1, T2>().pad(), [=](ampTiledIdx2D<T1, T2> tIdx) restrict(amp)
			{
				const ampIdx2D idx = tIdx.global;
				const int32 i1 = idx[0];
				const int32 i2 = idx[1];
				if (const int32 i1 = tIdx.global[0], i2 = tIdx.global[0]; i1 < cntY && i2 < cntX)
					function(tIdx);
			});
	}
	template<typename F>
	static void forEach(const int32 start, const int32 end, const F& function)
	{
		if (const int32 cnt = end - start; cnt > 0)
		{
			Concurrency::parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
			{
				const int32 i = tIdx.global[0] + start;
				if (i < end)
					function(i);
			});
		}
	}
	template <typename F>
	static void forEachTiled(const int32 cnt, const F& function)
	{
		if (!cnt) return;
		Concurrency::parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			if (tIdx.local[0] < cnt)
				function(tIdx.global[0], tIdx.tile[0], tIdx.local[0]);
		});
	}
	template <typename F>
	static void forEachTiledWithBarrier(const int32 cnt, const F& function)
	{
		if (!cnt) return;
		Concurrency::parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			function(tIdx);
		});
	}

	static uint32 reduceFlags(const ampArrayView<const uint32>& a, const int32 cnt)
	{
		if (!cnt) return 0;
		ampArrayView<uint32> flagBits(32);
		fill(flagBits, 0u);
		Concurrency::parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			const int32 gi = tIdx.global[0];
			const int32 li = tIdx.local[0];
			tile_static int32 lFlagBits[32];
			if (li < 32)
				lFlagBits[li] = 0;
			tIdx.barrier.wait_with_tile_static_memory_fence();
			if (gi < cnt)
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

	template<int32 TileSize> inline bool lastInTile(const ampTiledIdx<TileSize>& tIdx) restrict(amp)
	{
		return tIdx.local[0] == (TileSize - 1);
	}

	//namespace _reduce_detail
	//{
	//	template<int32 TileSize, typename T>
	//	ampArray<T> reduceInsideTiles(const ampArrayView<const T> src, ampArrayView<T> dst, int32 cnt = 0)
	//	{
	//		using namespace concurrency;
	//		if (!cnt) cnt = src.extent.size();
	//		const auto computeDomain = tileAndPad<TileSize>(cnt);
	//		ampArray<T> tileSums(computeDomain.size() / TileSize);
	//		parallel_for_each(computeDomain, [=, &tileSums](ampTiledIdx<TileSize> tIdx) restrict(amp)
	//		{
	//			const uint32 gi = tIdx.global[0], li = tIdx.local[0];
	//			const bool inRange = gi < cnt;

	//			tile_static T lPrefixSums[TileSize][2];
	//			const T origVal = inRange ? src[gi] : 0;
	//			lPrefixSums[li][0] = origVal;
	//			tIdx.barrier.wait_with_tile_static_memory_fence();

	//			for (uint32 i = 0; i < 8; i++)
	//			{
	//				const uint32 pow2i = _prefix_sum_detail::pow2(i);
	//				const uint32 w = (i + 1) % 2, r = i % 2;
	//				lPrefixSums[li][w] = (li >= pow2i) ?
	//					(lPrefixSums[li][r] + lPrefixSums[li - pow2i][r]) : lPrefixSums[li][r];
	//				tIdx.barrier.wait_with_tile_static_memory_fence();
	//			}
	//			T scan = (li == 0) ? 0 : lPrefixSums[li - 1][0];
	//			if (inRange) dst[gi] = scan;

	//			if (lastInTile(tIdx)) tileSums[tIdx.tile] = scan + origVal;
	//		});
	//		return tileSums;
	//	}
	//
	//	template<int TileSize, typename T>
	//	ampCopyFuture reduce(ampArray<T>& prefixSumsR, int32& sum, int32 cnt = 0)
	//	{
	//		if (!cnt) cnt = prefixSumsR.extent.size();
	//		unsigned long iterations;
	//		_BitScanReverse(&iterations, cnt);
	//		if (1 << iterations != cnt) iterations++;

	//		using namespace Concurrency;
	//		auto computeDomain = tileAndPad<TileSize>(cnt);
	//		ampArray<T> prefixSumsW(prefixSumsR.extent, prefixSumsR.accelerator_view);
	//		for (uint32 i = 0; i < iterations; i++)
	//		{
	//			const uint32 pow2i = _prefix_sum_detail::pow2(i);
	//			parallel_for_each(computeDomain, [=, &prefixSumsR, &prefixSumsW](ampTiledIdx<TileSize> tIdx) restrict(amp)
	//			{
	//				const int32 gi = tIdx.global[0];
	//				if (gi > cnt) return;
	//				prefixSumsW[gi] = (gi >= pow2i) ?
	//					(prefixSumsR[gi] + prefixSumsR[gi - pow2i]) : prefixSumsR[gi];
	//			});
	//			if (i != iterations - 1)
	//				std::swap(prefixSumsR, prefixSumsW);
	//		}
	//		ampCopyFuture fut = copyAsync(prefixSumsW, cnt - 1, sum);

	//		parallel_for_each(computeDomain, [=, &prefixSumsR, &prefixSumsW](ampTiledIdx<TileSize> tIdx) restrict(amp)
	//		{
	//			const int32 gi = tIdx.global[0];
	//			prefixSumsR[gi] = (gi == 0) ? 0 : prefixSumsW[gi - 1];
	//		});
	//		return fut;
	//	}
	//};

	//template<typename F>
	//static int32 reduce(const ampArrayView<const int32>& src, const int32 cnt, const F& function)
	//{
	//	if (!cnt) return 0;
	//	ampArray<int32> lPrefixSums(cnt);
	//	int32 lSum = 0, tSum = 0, ttSum = 0;

	//	Timer t = Timer();

	//	getGpuAccelView().wait();
	//	ampArray<int32> tPrefixSums = _reduce_detail::reduceInsideTiles<TILE_SIZE>(src,
	//		ampArrayView<int32>(lPrefixSums), cnt);
	//	getGpuAccelView().wait();
	//	ampCopyFuture lSumProm = copyAsync(lPrefixSums, cnt - 1, lSum);
	//	float32 t0 = t.Restart();

	//	ampArray<int32> tDst(tPrefixSums.extent);
	//	ampArray<int32> ttPrefixSums = _reduce_detail::reduceInsideTiles<TILE_SIZE>(
	//		ampArrayView<const int32>(tPrefixSums), ampArrayView<int32>(tDst));
	//	getGpuAccelView().wait();
	//	ampCopyFuture tSumProm = copyAsync(tDst, tDst.extent.size() - 1, tSum);
	//	float32 t1 = t.Restart();

	//	ampCopyFuture ttSumProm = _reduce_detail::reduce<TILE_SIZE>(ttPrefixSums, ttSum);
	//	getGpuAccelView().wait();
	//	float32 t2 = t.Restart();

	//	Concurrency::parallel_for_each(ampExtent(cnt).tile<TILE_SIZE>().pad(),
	//		[=, &lPrefixSums, &tPrefixSums, &ttPrefixSums](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
	//	{
	//		const int32 gi = tIdx.global[0];
	//		if (gi < cnt)
	//			function(gi, ttPrefixSums[tIdx.tile / TILE_SIZE] + tPrefixSums[tIdx.tile] + lPrefixSums[gi]);
	//	});
	//	getGpuAccelView().wait();
	//	float32 t3 = t.Stop();

	//	lSumProm.wait();
	//	tSumProm.wait();
	//	ttSumProm.wait();
	//	return ttSum + tSum + lSum;
	//	//snprintf(debugString, 64, "1: %f 2: %f 3: %f", t0, t1, t2);
	//}

	namespace _scan_detail
	{
		template <int TileSize, typename T>
		void tilewiseScan(const ampArrayView<const T>& input,
			ampArray<T>& tilewiseScans, ampArray<T>& tileSums, int32 cnt)
		{
			ampArrayView<T> tileSumsView(tileSums);
			ampArrayView<T> tilewiseScansView(tilewiseScans);
			const int32 tileCnt = (cnt + TileSize - 1) / TileSize;
			const int32 threadCnt = tileCnt * TileSize;

			parallel_for_each(tileAndPad<TileSize>(cnt),
				[=](ampTiledIdx<TileSize> tIdx) restrict(amp)
			{
				const uint32 li = tIdx.local[0];
				const int32 gi = tIdx.global[0];
				const bool inRange = gi < cnt;

				tile_static T tile[TileSize];
				T val = input[gi];
				if (inRange) tile[li] = val;

				for (uint32 offset = 2, half = 1; half <= TileSize; half = offset, offset *= 2)
				{
					tIdx.barrier.wait_with_tile_static_memory_fence();
					if ((li % offset) + 1 == offset)
						tile[li] += tile[li - half];
				}
				if (li % 2) val = tile[li];
				if (li == TileSize - 1)
					tileSumsView[tIdx.tile[0]] = val;

				tIdx.barrier.wait_with_tile_static_memory_fence();
				if (!inRange) return;

				for (uint32 offset = TileSize, half = TileSize / 2; half >= 2; offset = half, half /= 2)
					if (const uint32 modIdx = (li % offset) + 1; half < modIdx && modIdx < offset)
						val += tile[li + half - modIdx];
				tilewiseScansView[gi] = val;
			});
		}

		// Compute prefix of prefix
		template <int TileSize, typename T>
		void prefixScan(const ampArrayView<T>& a, int32 cnt)
		{
			ampArray<T> atemp(cnt);
			_scan_detail::scanTiled<TileSize>(ampArrayView<const T>(a), atemp, cnt);
			Concurrency::copy(atemp, a);
		}

		template <int TileSize, typename T>
		void scanTiled(const ampArrayView<const T>& input, ampArray<T>& output, int32 cnt)
		{
			const int32 tileCnt = (cnt + TileSize - 1) / TileSize;

			// Compute tile-wise scans and reductions
			ampArray<T> tileSumScan(tileCnt);
			_scan_detail::tilewiseScan<TileSize>(ampArrayView<const T>(input), output, tileSumScan, cnt);

			// recurse if necessary
			if (tileCnt > 1)
			{
				_scan_detail::prefixScan<TileSize>(ampArrayView<T>(tileSumScan), tileSumScan.extent[0]);

				if (cnt > 0)
				{
					ampArrayView<T> outputView(output);
					parallel_for_each(ampExtent(cnt), [=, &tileSumScan](ampIdx idx) restrict(amp)
					{
						const int32 tileIdx = idx[0] / TileSize;
						outputView[idx] = (tileIdx == 0) ?
							outputView[idx] : tileSumScan[tileIdx - 1] + outputView[idx];
					});
				}
			}

		}
	}

	template<typename T, typename F>
	int32 scan(const ampArrayView<const T>& src, const int32 cnt, const F& function)
	{
		ampArray<T> dst(cnt);
		//Timer t = Timer();
		_scan_detail::scanTiled<TILE_SIZE>(src, dst, cnt);

		T sum;
		ampCopyFuture sumFut = copyAsync(dst, cnt - 1, sum);
		//float32 t0 = t.Restart();

		parallel_for_each(tileAndPad<TILE_SIZE>(cnt), [=, &dst](ampTiledIdx<TILE_SIZE> tIdx) restrict(amp)
		{
			const int32 gi = tIdx.global[0];
			if (gi < cnt)
				function(gi, (gi == 0) ? 0 : dst[gi - 1]);
		});
		//float32 t2 = t.Stop();
		sumFut.wait();
		return sum;
	}


	namespace _prefix_sum_detail
	{
		inline uint32 pow2(uint32 x) restrict(amp) { return 1u << x; }
		inline uint32 pow2(uint32 x) { return 1u << x; }

		template<int TileSize>
		inline uint32 tileSum(uint32 x, ampTiledIdx<TileSize>& tIdx) restrict(amp)
		{
			uint32 li = tIdx.local[0];
			tile_static uint32 lSums[TileSize][2];

			lSums[li][0] = x;
			tIdx.barrier.wait_with_tile_static_memory_fence();

			for (uint32 i = 0; i < 8; i++)
			{
				if (li < _prefix_sum_detail::pow2(7 - i))
				{
					uint32 w = (i + 1) % 2;
					uint32 r = i % 2;

					lSums[li][w] = lSums[li * 2][r] + lSums[li * 2 + 1][r];
				}
				tIdx.barrier.wait_with_tile_static_memory_fence();
			}
			return lSums[0][0];
		}


		template<typename T, int TileSize>
		inline T tilePrefixSum(T val, ampTiledIdx<TileSize> tIdx, T& sum) restrict(amp)
		{
			const uint32 li = tIdx.local[0];

			tile_static T tile[TileSize];
			const T origVal = tile[li] = val;

			for (uint32 offset = 2, half = 1; half <= TileSize; half = offset, offset *= 2)
			{
				tIdx.barrier.wait_with_tile_static_memory_fence();
				if ((li % offset) + 1 == offset)
					tile[li] += tile[li - half];
			}
			if (li % 2) val = tile[li];

			tIdx.barrier.wait_with_tile_static_memory_fence();
			sum = tile[TileSize - 1];
			for (uint32 offset = TileSize, half = TileSize / 2; half >= 2; offset = half, half /= 2)				
				if (const uint32 modIdx = (li % offset) + 1; half < modIdx && modIdx < offset)
					val += tile[li + half - modIdx];
			return val - origVal;
			
			//const uint32 li = tIdx.local[0];
			//tile_static T lPrefixSums[TileSize][2];

			//lPrefixSums[li][0] = val;
			//tIdx.barrier.wait_with_tile_static_memory_fence();

			//for (uint32 i = 0; i < 8; i++)
			//{
			//	const uint32 pow2i = _prefix_sum_detail::pow2(i);

			//	const uint32 w = (i + 1) % 2;
			//	const uint32 r = i % 2;

			//	lPrefixSums[li][w] = (li >= pow2i) ?
			//		(lPrefixSums[li][r] + lPrefixSums[li - pow2i][r]) : lPrefixSums[li][r];

			//	tIdx.barrier.wait_with_tile_static_memory_fence();
			//}
			//sum = lPrefixSums[TileSize - 1][0];

			//return (li == 0) ? 0 : lPrefixSums[li - 1][0];
		}

		template<int TileSize>
		inline uint32 tilePrefixSum(uint32 x, ampTiledIdx<TileSize> tIdx) restrict(amp)
		{
			uint32 ll = 0;
			return tilePrefixSum(x, tIdx, ll);
		}
	}

	namespace _radix_sort_detail
	{
		inline uint32 getBits(uint32 x, uint32 numbits, uint32 bitoffset) restrict(amp)
		{
			return (x >> bitoffset) & ~(~0u << numbits);
		}

		template<int TileSize>
		static void calcIntermSums(const uint32 bitoffset, const ampArrayView<const Proxy>& intermArr,
			ampArray<uint32>& intermSums, ampArray<uint32>& intermPrefixSums)
		{
			const uint32 QuarterTile = TileSize / 4;
			const auto computeDomain = intermArr.extent.tile<TileSize>().pad();
			const uint32 tileCnt = computeDomain.size() / TileSize;
			Concurrency::parallel_for_each(computeDomain, [=, &intermSums](ampTiledIdx<TileSize> tIdx) restrict(amp)
			{
				const bool inbound = (tIdx.global[0] < intermArr.extent[0]);
				uint32 num = (inbound) ? _radix_sort_detail::getBits(intermArr[tIdx.global[0]].tag, 2, bitoffset) :
					_radix_sort_detail::getBits(0xffffffff, 2, bitoffset);
				for (uint32 i = 0; i < 4; i++)
				{
					const uint32 sum = _prefix_sum_detail::tileSum<TileSize>(num == i, tIdx);
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
					const uint32 scan = _prefix_sum_detail::tilePrefixSum(num, tIdx, lastVal0);
					if (gi < tileCnt4) intermPrefixSums[gi] = scan + lastVal1;

					lastVal1 += lastVal0;
				}
			});
		}

		template<int TileSize>
		static void radixSortStep(const uint32 bitoffset,
			const ampArrayView<const Proxy>& src, const ampArrayView<Proxy>& dest,
			ampArray<uint32>& intermPrefixSums)
		{
			const auto computeDomain = src.extent.tile<TileSize>().pad();
			const uint32 tileCnt = computeDomain.size() / TileSize;
			Concurrency::parallel_for_each(computeDomain, [=, &intermPrefixSums](ampTiledIdx<TileSize> tidx) restrict(amp)
			{
				const int32 gi = tidx.global[0];
				const bool inbounds = (gi < src.extent[0]);
				const Proxy element = inbounds ? src[gi] : Proxy(0, 0xffffffff);
				uint32 num = _radix_sort_detail::getBits(element.tag, 2, bitoffset);
				for (uint32 i = 0; i < 4; i++)
				{
					const uint32 scan = _prefix_sum_detail::tilePrefixSum((num == i), tidx)
						+ intermPrefixSums[i * tileCnt + tidx.tile[0]];
					if (num == i && inbounds) dest[scan] = element;
				}
			});
		}
	}
	static void radixSort(ampArrayView<Proxy>& a, const uint32 size)
	{
		const int32 tileCnt = getTileCnt<TILE_SIZE>(size);
		ampArray<Proxy> intermArr(size);
		ampArray<uint32> intermSums(tileCnt * 4);
		ampArray<uint32> intermPrefixSums(tileCnt * 4);

		ampArrayView<Proxy> av = a.section(0, size);
		for (uint32 i = 0; i < 16; i++)
		{
			const ampArrayView<const Proxy>& src = (i % 2 == 0) ? av : intermArr;
			const ampArrayView<Proxy>& dest = (i % 2 == 0) ? intermArr : av;

			const uint32 bitoffset = i * 2;
			_radix_sort_detail::calcIntermSums<TILE_SIZE>(bitoffset, src, intermSums, intermPrefixSums);
			_radix_sort_detail::radixSortStep<TILE_SIZE>(bitoffset, src, dest, intermPrefixSums);
		}
	}

	static void uninitialize()
	{
		concurrency::amp_uninitialize();
	}

	inline uint32 atomicAdd(uint32& dest, const uint32 add) restrict(amp)
	{
		return Concurrency::atomic_fetch_add(&dest, add);
	}
	inline int32 atomicAdd(int32& dest, const int32 add) restrict(amp)
	{
		return Concurrency::atomic_fetch_add(&dest, add);
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
	inline void atomicAdd(Vec2& dest, const Vec2& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
	}
	inline void atomicSub(Vec2& dest, const Vec2& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
	}
	inline void atomicAdd(Vec3& dest, const Vec2& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
	}
	inline void atomicSub(Vec3& dest, const Vec2& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
	}
	inline void atomicAdd(Vec3& dest, const Vec3& add) restrict(amp)
	{
		atomicAdd(dest.x, add.x);
		atomicAdd(dest.y, add.y);
		atomicAdd(dest.z, add.z);
	}
	inline void atomicSub(Vec3& dest, const Vec3& sub) restrict(amp)
	{
		atomicSub(dest.x, sub.x);
		atomicSub(dest.y, sub.y);
		atomicSub(dest.z, sub.z);
	}

	inline bool atomicAddFlag(uint32& dest, const uint32 flag) restrict(amp)
	{
		if (dest & flag) return false;
		return Concurrency::atomic_compare_exchange(&dest, &dest, dest | flag);
	}

	// returns value before increment
	inline int32 atomicInc(int32& dest) restrict(amp)
	{
		return Concurrency::atomic_fetch_inc(&dest);
	}
	inline int32 atomicInc(int32& dest, const int32 max) restrict(amp)
	{
		if (dest >= max) return 0;
		int32 expected = dest;
		int32 newValue = expected + 1;
		while (!Concurrency::atomic_compare_exchange(&dest, &expected, newValue))
		{
			newValue = expected + 1;
			if (newValue > max) return 0;
		}
		return newValue;
	}
};


class D11Device
{
private:
	ID3D11Device* pDevice = nullptr;
	ID3D11DeviceContext* pImmediateContext = nullptr;

	bool compareBuffers(ID3D11Buffer* pBuf1, ID3D11Buffer* pBuf2)
	{
		auto desc1 = getDescriptor(pBuf1);
		auto desc2 = getDescriptor(pBuf2);
		return desc1.ByteWidth == desc2.ByteWidth &&
			   desc1.Usage == desc2.Usage &&
			   desc1.BindFlags == desc2.BindFlags &&
			   desc1.CPUAccessFlags == desc2.CPUAccessFlags &&
			   desc1.MiscFlags == desc2.MiscFlags &&
			   desc1.StructureByteStride == desc2.StructureByteStride;
	}

public:
	D11Device()
	{
		pDevice = reinterpret_cast<ID3D11Device*>(Concurrency::direct3d::get_device(amp::getGpuAccelView()));
		pDevice->GetImmediateContext(&pImmediateContext);
	}
	~D11Device()
	{
		pDevice->Release();
	}

	void Flush() { if (pImmediateContext) pImmediateContext->Flush(); }

	template<typename T>
	bool copyRegion(ID3D11Buffer* pSrc, ID3D11Buffer* pDst, uint32 start, uint32 cnt)
	{
		if (!pSrc || !pDst || !cnt || !compareBuffers(pSrc, pDst)) return false;
		D3D11_BOX box{};
		box.left = start * sizeof(T);
		box.right = box.left + cnt * sizeof(T);
		box.top = box.front = 0;
		box.bottom = box.back = 1;
		pImmediateContext->CopySubresourceRegion(pDst, 0, box.left, 0, 0, pSrc, 0, &box);
		return true;
	}

	template <typename T>
	void copy(const ampArray<T>& src, ID3D11Buffer* pDst, const int32 cnt)
	{
		if (cnt <= 0 || !pDst) return;

		ID3D11Buffer* pSrc = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(src));
		if (copyRegion<T>(pSrc, pDst, 0, cnt))
			pImmediateContext->Flush();
		if (pSrc) pSrc->Release();
	}

	template <typename T>
	void copy(const ampArray<T>& src, ID3D11Buffer* pDst, const int32 start, const int32 end)
	{
		if (end <= start || !pDst) return;

		ID3D11Buffer* pSrc = reinterpret_cast<ID3D11Buffer*>(concurrency::direct3d::get_buffer(src));
		if (copyRegion<T>(pSrc, pDst, start, end - start))
			pImmediateContext->Flush();
		if (pSrc) pSrc->Release();
	}

	inline D3D11_BUFFER_DESC getDescriptor(ID3D11Buffer* pBuffer)
	{
		D3D11_BUFFER_DESC desc;
		pBuffer->GetDesc(&desc);
		return desc;
	}
};
