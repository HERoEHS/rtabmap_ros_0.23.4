#ifndef RTABMAP_SLAM_MEMORY_COMPATIBILITY_H_
#define RTABMAP_SLAM_MEMORY_COMPATIBILITY_H_

namespace rtabmap_slam
{
namespace detail
{
// Newer RTAB-Map cores require this argument. false preserves the older
// no-argument accessor's behavior: return the last signature, including
// intermediate nodes. Keep compatibility with the deployed older core too.
template<typename Memory>
auto lastWorkingSignature(const Memory * memory, int)
    -> decltype(memory->getLastWorkingSignature(false))
{
    return memory->getLastWorkingSignature(false);
}

template<typename Memory>
auto lastWorkingSignature(const Memory * memory, long)
    -> decltype(memory->getLastWorkingSignature())
{
    return memory->getLastWorkingSignature();
}
}  // namespace detail

template<typename Memory>
auto lastWorkingSignature(const Memory * memory)
    -> decltype(detail::lastWorkingSignature(memory, 0))
{
    return detail::lastWorkingSignature(memory, 0);
}
}  // namespace rtabmap_slam

#endif
