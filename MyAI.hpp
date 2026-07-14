// ======================================================================
// FILE:        MyAI.hpp
//
// AUTHOR:      Jian Li
//
// DESCRIPTION: This file contains your agent class, which you will
//              implement. You are responsible for implementing the
//              'getAction' function and any helper methods you feel you
//              need.
//
// NOTES:       - If you are having trouble understanding how the shell
//                works, look at the other parts of the code, as well as
//                the documentation.
//
//              - You are only allowed to make changes to this portion of
//                the code. Any changes to other portions of the code will
//                be lost when the tournament runs your code.
// ======================================================================
#ifndef MINE_SWEEPER_CPP_SHELL_MYAI_HPP
#define MINE_SWEEPER_CPP_SHELL_MYAI_HPP
#include "Agent.hpp"
#include <vector>
#include <deque>
#include <utility>
#include <cstdint>

using namespace std;

class MyAI : public Agent
{
public:
    MyAI ( int _rowDimension, int _colDimension, int _totalMines, int _agentX, int _agentY );
    Action getAction ( int number ) override;

    // ======================================================================
    // YOUR CODE BEGINS
    // ======================================================================
private:
    int rowDim;
    int colDim;
    int totalMines;

    enum CellState: unsigned char {
        COVERED= 0,
        UNCOVERED= 1,
        FLAGGED= 2
    };

    vector<vector<CellState>> state;
    vector<vector<int>> hint;

    int flaggedCount;
    int uncoveredCount;
    int totalCells;

    Action lastAction;

    deque<pair<int,int>> safeQueue;
    deque<pair<int,int>> mineQueue;

    vector<vector<bool>> inSafeQ;
    vector<vector<bool>> inMineQ;

    struct Constraint {
        vector<int> cells;
        int mines;
    };

    bool inBounds ( int x, int y ) const;
    void getNeighbors ( int x, int y, vector<pair<int,int>>& out ) const;
    int  cellId ( int x, int y ) const { return x * rowDim + y; }
    pair<int,int> idToXY ( int id ) const { return { id / rowDim, id % rowDim }; }

    void applyPercept ( int number );
    void propagateConstraints ( );
    void buildConstraints ( vector<Constraint>& out ) const;
    bool subsetRule ( );
    Action nextPlannedAction ( );

    Action enumeratedDecision ( );
    Action localProbabilityGuess ( );

    static const int COMPONENT_SIZE_CAP = 22;

    // ======================================================================
    // YOUR CODE ENDS
    // ======================================================================
};
#endif //MINE_SWEEPER_CPP_SHELL_MYAI_HPP