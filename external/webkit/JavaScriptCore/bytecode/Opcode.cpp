

#include "config.h"
#include "Opcode.h"

using namespace std;

namespace JSC {

#if ENABLE(OPCODE_SAMPLING) || ENABLE(CODEBLOCK_SAMPLING) || ENABLE(OPCODE_STATS)

const char* const opcodeNames[] = {
#define OPCODE_NAME_ENTRY(opcode, size) #opcode,
    FOR_EACH_OPCODE_ID(OPCODE_NAME_ENTRY)
#undef OPCODE_NAME_ENTRY
};

#endif

#if ENABLE(OPCODE_STATS)

long long OpcodeStats::opcodeCounts[numOpcodeIDs];
long long OpcodeStats::opcodePairCounts[numOpcodeIDs][numOpcodeIDs];
int OpcodeStats::lastOpcode = -1;

static OpcodeStats logger;

OpcodeStats::OpcodeStats()
{
    for (int i = 0; i < numOpcodeIDs; ++i)
        opcodeCounts[i] = 0;
    
    for (int i = 0; i < numOpcodeIDs; ++i)
        for (int j = 0; j < numOpcodeIDs; ++j)
            opcodePairCounts[i][j] = 0;
}

static int compareOpcodeIndices(const void* left, const void* right)
{
    long long leftValue = OpcodeStats::opcodeCounts[*(int*) left];
    long long rightValue = OpcodeStats::opcodeCounts[*(int*) right];
    
    if (leftValue < rightValue)
        return 1;
    else if (leftValue > rightValue)
        return -1;
    else
        return 0;
}

static int compareOpcodePairIndices(const void* left, const void* right)
{
    pair<int, int> leftPair = *(pair<int, int>*) left;
    long long leftValue = OpcodeStats::opcodePairCounts[leftPair.first][leftPair.second];
    pair<int, int> rightPair = *(pair<int, int>*) right;
    long long rightValue = OpcodeStats::opcodePairCounts[rightPair.first][rightPair.second];
    
    if (leftValue < rightValue)
        return 1;
    else if (leftValue > rightValue)
        return -1;
    else
        return 0;
}

OpcodeStats::~OpcodeStats()
{
    long long totalInstructions = 0;
    for (int i = 0; i < numOpcodeIDs; ++i)
        totalInstructions += opcodeCounts[i];
    
    long long totalInstructionPairs = 0;
    for (int i = 0; i < numOpcodeIDs; ++i)
        for (int j = 0; j < numOpcodeIDs; ++j)
            totalInstructionPairs += opcodePairCounts[i][j];

    int sortedIndices[numOpcodeIDs];    
    for (int i = 0; i < numOpcodeIDs; ++i)
        sortedIndices[i] = i;
    qsort(sortedIndices, numOpcodeIDs, sizeof(int), compareOpcodeIndices);
    
    pair<int, int> sortedPairIndices[numOpcodeIDs * numOpcodeIDs];
    pair<int, int>* currentPairIndex = sortedPairIndices;
    for (int i = 0; i < numOpcodeIDs; ++i)
        for (int j = 0; j < numOpcodeIDs; ++j)
            *(currentPairIndex++) = make_pair(i, j);
    qsort(sortedPairIndices, numOpcodeIDs * numOpcodeIDs, sizeof(pair<int, int>), compareOpcodePairIndices);
    
    printf("\nExecuted opcode statistics\n"); 
    
    printf("Total instructions executed: %lld\n\n", totalInstructions);

    printf("All opcodes by frequency:\n\n");

    for (int i = 0; i < numOpcodeIDs; ++i) {
        int index = sortedIndices[i];
        printf("%s:%s %lld - %.2f%%\n", opcodeNames[index], padOpcodeName((OpcodeID)index, 28), opcodeCounts[index], ((double) opcodeCounts[index]) / ((double) totalInstructions) * 100.0);    
    }
    
    printf("\n");
    printf("2-opcode sequences by frequency: %lld\n\n", totalInstructions);
    
    for (int i = 0; i < numOpcodeIDs * numOpcodeIDs; ++i) {
        pair<int, int> indexPair = sortedPairIndices[i];
        long long count = opcodePairCounts[indexPair.first][indexPair.second];
        
        if (!count)
            break;
        
        printf("%s%s %s:%s %lld %.2f%%\n", opcodeNames[indexPair.first], padOpcodeName((OpcodeID)indexPair.first, 28), opcodeNames[indexPair.second], padOpcodeName((OpcodeID)indexPair.second, 28), count, ((double) count) / ((double) totalInstructionPairs) * 100.0);
    }
    
    printf("\n");
    printf("Most common opcodes and sequences:\n");

    for (int i = 0; i < numOpcodeIDs; ++i) {
        int index = sortedIndices[i];
        long long opcodeCount = opcodeCounts[index];
        double opcodeProportion = ((double) opcodeCount) / ((double) totalInstructions);
        if (opcodeProportion < 0.0001)
            break;
        printf("\n%s:%s %lld - %.2f%%\n", opcodeNames[index], padOpcodeName((OpcodeID)index, 28), opcodeCount, opcodeProportion * 100.0);

        for (int j = 0; j < numOpcodeIDs * numOpcodeIDs; ++j) {
            pair<int, int> indexPair = sortedPairIndices[j];
            long long pairCount = opcodePairCounts[indexPair.first][indexPair.second];
            double pairProportion = ((double) pairCount) / ((double) totalInstructionPairs);
        
            if (!pairCount || pairProportion < 0.0001 || pairProportion < opcodeProportion / 100)
                break;

            if (indexPair.first != index && indexPair.second != index)
                continue;

            printf("    %s%s %s:%s %lld - %.2f%%\n", opcodeNames[indexPair.first], padOpcodeName((OpcodeID)indexPair.first, 28), opcodeNames[indexPair.second], padOpcodeName((OpcodeID)indexPair.second, 28), pairCount, pairProportion * 100.0);
        }
        
    }
    printf("\n");
}

void OpcodeStats::recordInstruction(int opcode)
{
    opcodeCounts[opcode]++;
    
    if (lastOpcode != -1)
        opcodePairCounts[lastOpcode][opcode]++;
    
    lastOpcode = opcode;
}

void OpcodeStats::resetLastInstruction()
{
    lastOpcode = -1;
}

#endif

} // namespace JSC
