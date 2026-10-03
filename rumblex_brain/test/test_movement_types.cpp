#include <gtest/gtest.h>

#include "requester/utility.hpp"

const void* movementTypeMapFromOtherTranslationUnit();
const void* movementNameMapFromOtherTranslationUnit();

TEST(MovementTypes, MapsAreSharedAcrossTranslationUnits) {
    EXPECT_EQ(&brain::movementTypeToName, movementTypeMapFromOtherTranslationUnit());
    EXPECT_EQ(&brain::nameToMovementType, movementNameMapFromOtherTranslationUnit());
}

TEST(MovementTypes, EveryMovementNameRoundTrips) {
    ASSERT_FALSE(brain::movementTypeToName.empty());
    ASSERT_EQ(brain::movementTypeToName.size(), brain::nameToMovementType.size());
    for (const auto& [type, name] : brain::movementTypeToName) {
        EXPECT_EQ(brain::nameToMovementType.at(name), type);
    }
}
