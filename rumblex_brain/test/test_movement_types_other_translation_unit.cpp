#include "requester/utility.hpp"

const void* movementTypeMapFromOtherTranslationUnit() {
    return &brain::movementTypeToName;
}

const void* movementNameMapFromOtherTranslationUnit() {
    return &brain::nameToMovementType;
}
