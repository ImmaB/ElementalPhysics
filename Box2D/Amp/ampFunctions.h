#pragma once

#include <amp.h>
#include <memory>
#include <type_traits>

namespace amp
{
    namespace details
    {
        // This struct may be used as an example of creating a new sort key
        // type, but should not be specified directly. When key_index_type
        // returns an instance of this, it is assumed to be redundant and
        // sort keys are not used.
        template<typename SimpleType>
        struct SimpleKeyIndex {
            SimpleType k;
            int i;
            SimpleKeyIndex() restrict(cpu, amp) { }
            SimpleKeyIndex(SimpleType key, int index) restrict(cpu, amp)
                : k(key), i(index) { }
            bool operator<(const SimpleKeyIndex& other) restrict(cpu, amp) {
                return k < other.k;
            }
        };
    }

    // Specialisations of this struct should be used to provide type-specific
    // sort keys. An example may look like:
    //
    //     template<> struct bitonic_sort::key_index_type<UDT> {
    //         typedef UDTKeyIndex type;
    //     };
    template<typename T>
    struct key_index_type { 
        typedef details::SimpleKeyIndex<T> type;
    };

    namespace details
    {
        template<typename T>
        struct uses_sort_key {
            static const bool value = !std::is_same<
                typename key_index_type<T>::type, SimpleKeyIndex<T>
            >::value;
        };

        template<bool First, typename KeyType>
        void parallel_sort_phase(int virtual_size, int block_size,
                                concurrency::array<KeyType, 1>& dest) {
            const int actual_size = dest.extent.size();
            const int half_block_size = block_size / 2;
            const int mask = block_size - 1;
            const int half_mask = half_block_size - 1;
            concurrency::parallel_for_each(dest.accelerator_view,
                concurrency::extent<1>(virtual_size / 2),
                [=, &dest](concurrency::index<1> i) restrict(amp) {
                const int i1 = ((i[0] & ~half_mask) << 1) | (i[0] & half_mask);
                const int i2 = (First) ? ((i1 | mask) - (i1 & mask)) : (i1 + half_block_size);

                if (i2 < actual_size && !(dest[i1] < dest[i2])) {
                    auto temp = dest[i1];
                    dest[i1] = dest[i2];
                    dest[i2] = temp;
                }
            });
        }

        template<typename KeyType>
        void parallel_sort_inplace(concurrency::array<KeyType, 1>& dest) {
            const int actual_size = dest.extent.size();
            int virt_size = 1, phase_count = 0;
            while (virt_size < actual_size) {
                virt_size *= 2;
                phase_count += 1;
            }
            
            int starting_block_size = 2;
            while (--phase_count >= 0) {
                int current_block_size = starting_block_size;
                starting_block_size *= 2;

                details::parallel_sort_phase<true>(virt_size, current_block_size, dest);
            
                while ((current_block_size /= 2) >= 2) {
                    details::parallel_sort_phase<false>(virt_size, current_block_size, dest);
                }
            }
        }
    }

    template<typename SourceType>
    std::shared_ptr<concurrency::array<typename key_index_type<SourceType>::type, 1>>
    parallel_sort_keys(const concurrency::array<SourceType, 1>& source) {
        const int actual_size = source.extent.size();

        typedef typename key_index_type<SourceType>::type Key;
        auto pKeys = std::make_shared<concurrency::array<Key, 1>>(
            actual_size,
            source.accelerator_view
        );
        auto& keys = *pKeys;
        concurrency::parallel_for_each(keys.accelerator_view, keys.extent,
            [=, &keys, &source](concurrency::index<1> i) restrict(amp) {
            keys[i] = Key(source[i], i[0]);
        });

        details::parallel_sort_inplace(keys);

        return pKeys;
    }

    template<typename SourceType>
    typename std::enable_if<
        details::uses_sort_key<SourceType>::value,
        std::shared_ptr<concurrency::array<SourceType, 1>>
    >::type
    sort(const concurrency::array<SourceType, 1>& source, bool reverse=false) {
        auto pKeys = parallel_sort_keys(source);
        auto& keys = *pKeys;

        auto pResult = std::make_shared<concurrency::array<SourceType, 1>>(
            source.extent.size(),
            source.accelerator_view);
        auto& result = *pResult;

        if (!reverse) {
            concurrency::parallel_for_each(result.accelerator_view, result.extent,
                [&](concurrency::index<1> i) restrict(amp) {
                result[i] = source[keys[i].i];
            });
        } else {
            concurrency::parallel_for_each(result.accelerator_view, result.extent,
                [&](concurrency::index<1> i) restrict(amp) {
                result[i] = source[keys[keys.extent.size() - 1 - i].i];
            });
        }
        return pResult;
    }

    template<typename SourceType>
    typename std::enable_if<
        !details::uses_sort_key<SourceType>::value,
        std::shared_ptr<concurrency::array<SourceType, 1>>
    >::type
    sort(const concurrency::array<SourceType, 1>& source, bool reverse=false) {
        auto pResult = std::make_shared<concurrency::array<SourceType, 1>>(source);
        auto& result = *pResult;
        
        details::parallel_sort_inplace(result);

        if (reverse) {
            auto pNewResult = std::make_shared<concurrency::array<SourceType, 1>>(
                result.extent.size(),
                result.accelerator_view);
            auto& newResult = *pNewResult;
            concurrency::parallel_for_each(result.accelerator_view, result.extent,
                [&](concurrency::index<1> i) restrict(amp) {
                newResult[i] = result[result.extent.size() - 1 - i];
            });

            return pNewResult;
        }

        return pResult;
    }
}
