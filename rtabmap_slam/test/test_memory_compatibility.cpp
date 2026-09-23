#include "rtabmap_slam/MemoryCompatibility.h"

struct Signature {};
struct OldMemory
{
    Signature last;
    const Signature * getLastWorkingSignature() const { return &last; }
};
struct NewMemory
{
    Signature intermediate;
    Signature previous;
    const Signature * getLastWorkingSignature(bool ignoreIntermediateNodes) const
    {
        return ignoreIntermediateNodes ? &previous : &intermediate;
    }
};

int main()
{
    const OldMemory oldMemory;
    const NewMemory newMemory;
    if(rtabmap_slam::lastWorkingSignature(&oldMemory) != &oldMemory.last)
        return 1;
    // A newer core must still return an intermediate last signature, as the
    // old accessor did. Passing true would silently change map behavior.
    if(rtabmap_slam::lastWorkingSignature(&newMemory) != &newMemory.intermediate)
        return 2;
    return 0;
}
