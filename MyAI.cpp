// ======================================================================
// FILE:        MyAI.cpp
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

#include "MyAI.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

MyAI::MyAI ( int _rowDimension, int _colDimension, int _totalMines, int _agentX, int _agentY )
    : Agent(),
      rowDim(_rowDimension),
      colDim(_colDimension),
      totalMines(_totalMines),
      flaggedCount(0),
      uncoveredCount(0),
      totalCells(_rowDimension * _colDimension)
{
    // ======================================================================
    // YOUR CODE BEGINS
    // ======================================================================
    state.assign(colDim, vector<CellState>(rowDim, COVERED));
    hint .assign(colDim, vector<int>(rowDim, 0));
    inSafeQ.assign(colDim, vector<bool>(rowDim, false));
    inMineQ.assign(colDim, vector<bool>(rowDim, false));

    lastAction = { UNCOVER, _agentX, _agentY };
    // ======================================================================
    // YOUR CODE ENDS
    // ======================================================================
};

Agent::Action MyAI::getAction( int number )
{
    // ======================================================================
    // YOUR CODE BEGINS
    // ======================================================================
    applyPercept(number);

    if ( uncoveredCount >= totalCells - totalMines )
    {
        lastAction = { LEAVE, -1, -1 };
        return lastAction;
    }

    propagateConstraints();

    if ( safeQueue.empty() && mineQueue.empty() )
    {
        while ( subsetRule() )
            propagateConstraints();
    }

    lastAction = nextPlannedAction();

    if ( lastAction.action == LEAVE )
    {
        Action e = enumeratedDecision();
        if ( e.action != LEAVE )
        {
            lastAction = e;
        }
        else
        {
            Action g = localProbabilityGuess();
            if ( g.action != LEAVE )
                lastAction = g;
        }
    }

    return lastAction;
    // ======================================================================
    // YOUR CODE ENDS
    // ======================================================================
}

// ======================================================================
// YOUR CODE BEGINS
// ======================================================================
bool MyAI::inBounds ( int x, int y ) const
{
    return x >= 0 && x < colDim && y >= 0 && y < rowDim;
}

void MyAI::getNeighbors ( int x, int y, vector<pair<int,int>>& out ) const
{
    out.clear();
    for ( int dx = -1; dx <= 1; ++dx )
    {
        for ( int dy = -1; dy <= 1; ++dy )
        {
            if ( dx == 0 && dy == 0 ) continue;
            int nx = x + dx, ny = y + dy;
            if ( inBounds(nx, ny) )
                out.emplace_back(nx, ny);
        }
    }
}

void MyAI::applyPercept ( int number )
{
    int x = lastAction.x;
    int y = lastAction.y;
    if ( !inBounds(x, y) ) return;

    switch ( lastAction.action )
    {
        case UNCOVER:
            if ( state[x][y] != UNCOVERED )
            {
                state[x][y] = UNCOVERED;
                hint [x][y] = number;
                ++uncoveredCount;
            }
            inSafeQ[x][y] = false;
            break;

        case FLAG:
            if ( state[x][y] != FLAGGED )
            {
                state[x][y] = FLAGGED;
                ++flaggedCount;
            }
            inMineQ[x][y] = false;
            break;

        case UNFLAG:
            if ( state[x][y] == FLAGGED )
            {
                state[x][y] = COVERED;
                --flaggedCount;
            }
            break;

        case LEAVE:
        default:
            break;
    }
}

void MyAI::propagateConstraints ( )
{
    vector<pair<int,int>> nbrs;
    nbrs.reserve(8);

    bool progress = true;
    while ( progress )
    {
        progress = false;

        if ( flaggedCount == totalMines )
        {
            for ( int x = 0; x < colDim; ++x )
            {
                for ( int y = 0; y < rowDim; ++y )
                {
                    if ( state[x][y] == COVERED && !inSafeQ[x][y] )
                    {
                        safeQueue.emplace_back(x, y);
                        inSafeQ[x][y] = true;
                        progress = true;
                    }
                }
            }
            if ( progress ) continue;
        }

        for ( int x = 0; x < colDim; ++x )
        {
            for ( int y = 0; y < rowDim; ++y )
            {
                if ( state[x][y] != UNCOVERED ) continue;
                int h = hint[x][y];
                if ( h < 0 ) continue;

                getNeighbors(x, y, nbrs);

                int flagged = 0;
                int coveredUnflagged = 0;
                for ( auto& p : nbrs )
                {
                    CellState s = state[p.first][p.second];
                    if      ( s == FLAGGED ) ++flagged;
                    else if ( s == COVERED ) ++coveredUnflagged;
                }

                if ( coveredUnflagged == 0 ) continue;

                int remaining = h - flagged;

                if ( remaining == 0 )
                {
                    for ( auto& p : nbrs )
                    {
                        if ( state[p.first][p.second] == COVERED
                             && !inSafeQ[p.first][p.second] )
                        {
                            safeQueue.emplace_back(p.first, p.second);
                            inSafeQ[p.first][p.second] = true;
                            progress = true;
                        }
                    }
                }
                else if ( remaining == coveredUnflagged )
                {
                    for ( auto& p : nbrs )
                    {
                        if ( state[p.first][p.second] == COVERED
                             && !inMineQ[p.first][p.second] )
                        {
                            mineQueue.emplace_back(p.first, p.second);
                            inMineQ[p.first][p.second] = true;
                            progress = true;
                        }
                    }
                }
            }
        }
    }
}

void MyAI::buildConstraints ( vector<Constraint>& out ) const
{
    out.clear();
    vector<pair<int,int>> nbrs;
    nbrs.reserve(8);

    for ( int x = 0; x < colDim; ++x )
    {
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[x][y] != UNCOVERED ) continue;
            int h = hint[x][y];
            if ( h <= 0 ) continue;

            const_cast<MyAI*>(this)->getNeighbors(x, y, nbrs);

            Constraint c;
            int flagged = 0;
            for ( auto& p : nbrs )
            {
                CellState s = state[p.first][p.second];
                if      ( s == FLAGGED ) ++flagged;
                else if ( s == COVERED ) c.cells.push_back(cellId(p.first, p.second));
            }

            if ( c.cells.empty() ) continue;

            c.mines = h - flagged;
            if ( c.mines < 0 || c.mines > (int)c.cells.size() ) continue;

            sort(c.cells.begin(), c.cells.end());
            out.push_back(move(c));
        }
    }

    sort(out.begin(), out.end(), [](const Constraint& a, const Constraint& b){
        if ( a.cells.size() != b.cells.size() ) return a.cells.size() < b.cells.size();
        return a.cells < b.cells;
    });
    vector<Constraint> dedup;
    dedup.reserve(out.size());
    for ( auto& c : out )
    {
        if ( !dedup.empty()
             && dedup.back().cells == c.cells
             && dedup.back().mines == c.mines )
            continue;
        dedup.push_back(move(c));
    }
    out = move(dedup);
}

bool MyAI::subsetRule ( )
{
    vector<Constraint> cs;
    buildConstraints(cs);
    if ( cs.size() < 2 ) return false;

    bool found = false;

    for ( size_t i = 0; i < cs.size(); ++i )
    {
        for ( size_t j = 0; j < cs.size(); ++j )
        {
            if ( i == j ) continue;
            const Constraint& A = cs[i];
            const Constraint& B = cs[j];
            if ( A.cells.size() >= B.cells.size() ) continue;

            size_t ai = 0, bi = 0;
            while ( ai < A.cells.size() && bi < B.cells.size() )
            {
                if ( A.cells[ai] == B.cells[bi] ) { ++ai; ++bi; }
                else if ( A.cells[ai] >  B.cells[bi] ) { ++bi; }
                else break;
            }
            if ( ai < A.cells.size() ) continue;

            vector<int> diff;
            diff.reserve(B.cells.size() - A.cells.size());
            ai = 0; bi = 0;
            while ( bi < B.cells.size() )
            {
                if ( ai < A.cells.size() && A.cells[ai] == B.cells[bi] ) { ++ai; ++bi; }
                else { diff.push_back(B.cells[bi]); ++bi; }
            }

            int diffMines = B.mines - A.mines;

            if ( diffMines == 0 )
            {
                for ( int id : diff )
                {
                    auto p = idToXY(id);
                    if ( state[p.first][p.second] == COVERED && !inSafeQ[p.first][p.second] )
                    {
                        safeQueue.emplace_back(p.first, p.second);
                        inSafeQ[p.first][p.second] = true;
                        found = true;
                    }
                }
            }
            else if ( diffMines == (int)diff.size() )
            {
                for ( int id : diff )
                {
                    auto p = idToXY(id);
                    if ( state[p.first][p.second] == COVERED && !inMineQ[p.first][p.second] )
                    {
                        mineQueue.emplace_back(p.first, p.second);
                        inMineQ[p.first][p.second] = true;
                        found = true;
                    }
                }
            }
        }
    }

    return found;
}

Agent::Action MyAI::nextPlannedAction ( )
{
    while ( !mineQueue.empty() )
    {
        auto p = mineQueue.front();
        mineQueue.pop_front();
        if ( state[p.first][p.second] != COVERED ) continue;
        inMineQ[p.first][p.second] = false;
        return { FLAG, p.first, p.second };
    }

    while ( !safeQueue.empty() )
    {
        auto p = safeQueue.front();
        safeQueue.pop_front();
        if ( state[p.first][p.second] != COVERED ) continue;
        inSafeQ[p.first][p.second] = false;
        return { UNCOVER, p.first, p.second };
    }

    return { LEAVE, -1, -1 };
}

static double binomial ( int n, int k )
{
    if ( k < 0 || k > n ) return 0.0;
    if ( k == 0 || k == n ) return 1.0;
    if ( k > n - k ) k = n - k;
    double r = 1.0;
    for ( int i = 0; i < k; ++i )
    {
        r *= (double)(n - i);
        r /= (double)(i + 1);
    }
    return r;
}

Agent::Action MyAI::enumeratedDecision ( )
{
    vector<Constraint> cs;
    buildConstraints(cs);
    if ( cs.empty() ) return { LEAVE, -1, -1 };

    vector<int> frontierCells;
    {
        vector<bool> seen(totalCells, false);
        for ( auto& c : cs )
            for ( int id : c.cells )
                if ( !seen[id] ) { seen[id] = true; frontierCells.push_back(id); }
    }
    sort(frontierCells.begin(), frontierCells.end());

    vector<int> cellToLocal(totalCells, -1);
    for ( size_t i = 0; i < frontierCells.size(); ++i )
        cellToLocal[ frontierCells[i] ] = (int)i;

    int nFrontier = (int)frontierCells.size();

    struct LConstraint {
        vector<int> cells;
        int         mines;
    };
    vector<LConstraint> lcs;
    lcs.reserve(cs.size());
    for ( auto& c : cs )
    {
        LConstraint lc;
        lc.cells.reserve(c.cells.size());
        for ( int id : c.cells ) lc.cells.push_back(cellToLocal[id]);
        sort(lc.cells.begin(), lc.cells.end());
        lc.mines = c.mines;
        lcs.push_back(move(lc));
    }

    vector<vector<int>> cellConstraints(nFrontier);
    for ( int ci = 0; ci < (int)lcs.size(); ++ci )
        for ( int loc : lcs[ci].cells )
            cellConstraints[loc].push_back(ci);

    vector<int> parent(nFrontier);
    for ( int i = 0; i < nFrontier; ++i ) parent[i] = i;
    function<int(int)> findp = [&](int x) {
        while ( parent[x] != x ) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite = [&](int a, int b) {
        a = findp(a); b = findp(b);
        if ( a != b ) parent[a] = b;
    };
    for ( auto& lc : lcs )
        for ( size_t k = 1; k < lc.cells.size(); ++k )
            unite(lc.cells[0], lc.cells[k]);

    vector<int> compOfCell(nFrontier, -1);
    vector<vector<int>> componentCells;
    for ( int i = 0; i < nFrontier; ++i )
    {
        int r = findp(i);
        if ( compOfCell[r] == -1 )
        {
            compOfCell[r] = (int)componentCells.size();
            componentCells.emplace_back();
        }
        compOfCell[i] = compOfCell[r];
        componentCells[compOfCell[i]].push_back(i);
    }
    int nComp = (int)componentCells.size();

    vector<vector<int>> componentConstraints(nComp);
    for ( int ci = 0; ci < (int)lcs.size(); ++ci )
    {
        int comp = compOfCell[ lcs[ci].cells[0] ];
        componentConstraints[comp].push_back(ci);
    }

    vector<vector<double>> compWaysAt(nComp);
    vector<vector<vector<double>>> compCellMineAt(nComp);
    vector<bool> compEnumerated(nComp, true);

    for ( int comp = 0; comp < nComp; ++comp )
    {
        const vector<int>& cells = componentCells[comp];
        int nc = (int)cells.size();

        if ( nc > COMPONENT_SIZE_CAP )
        {
            compEnumerated[comp] = false;
            continue;
        }

        vector<int> localToComp(nFrontier, -1);
        for ( int ii = 0; ii < nc; ++ii ) localToComp[cells[ii]] = ii;

        struct CCon {
            vector<int> idx;
            int         mines;
        };
        vector<CCon> ccons;
        ccons.reserve(componentConstraints[comp].size());
        for ( int ci : componentConstraints[comp] )
        {
            CCon cc;
            cc.idx.reserve(lcs[ci].cells.size());
            for ( int loc : lcs[ci].cells ) cc.idx.push_back(localToComp[loc]);
            sort(cc.idx.begin(), cc.idx.end());
            cc.mines = lcs[ci].mines;
            ccons.push_back(move(cc));
        }

        compWaysAt[comp].assign(nc + 1, 0.0);
        compCellMineAt[comp].assign(nc + 1, vector<double>(nc, 0.0));

        vector<int> assignment(nc, 0);

        int nCons = (int)ccons.size();
        vector<int> conSum(nCons, 0);
        vector<int> conUnassigned(nCons);
        for ( int ci = 0; ci < nCons; ++ci ) conUnassigned[ci] = (int)ccons[ci].idx.size();

        vector<vector<int>> cellCons(nc);
        for ( int ci = 0; ci < nCons; ++ci )
            for ( int idx : ccons[ci].idx )
                cellCons[idx].push_back(ci);

        function<void(int,int)> dfs = [&](int pos, int minesSoFar) {
            if ( pos == nc )
            {
                for ( int ci = 0; ci < nCons; ++ci )
                    if ( conSum[ci] != ccons[ci].mines ) return;

                compWaysAt[comp][minesSoFar] += 1.0;
                for ( int ii = 0; ii < nc; ++ii )
                    if ( assignment[ii] )
                        compCellMineAt[comp][minesSoFar][ii] += 1.0;
                return;
            }

            for ( int v = 0; v <= 1; ++v )
            {
                assignment[pos] = v;
                bool ok = true;

                for ( int ci : cellCons[pos] )
                {
                    conSum[ci] += v;
                    conUnassigned[ci] -= 1;
                    if ( conSum[ci] > ccons[ci].mines )
                    {
                        ok = false;
                    }
                    else if ( conSum[ci] + conUnassigned[ci] < ccons[ci].mines )
                    {
                        ok = false;
                    }
                }

                if ( ok ) dfs(pos + 1, minesSoFar + v);

                for ( int ci : cellCons[pos] )
                {
                    conSum[ci] -= v;
                    conUnassigned[ci] += 1;
                }
            }
        };

        dfs(0, 0);
    }

    int nonFrontierCovered = 0;
    for ( int x = 0; x < colDim; ++x )
    {
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[x][y] == COVERED && cellToLocal[ cellId(x, y) ] == -1 )
                ++nonFrontierCovered;
        }
    }

    int minesLeft = totalMines - flaggedCount;
    if ( minesLeft < 0 ) minesLeft = 0;

    vector<int> enumeratedComps, skippedComps;
    for ( int comp = 0; comp < nComp; ++comp )
    {
        if ( compEnumerated[comp] ) enumeratedComps.push_back(comp);
        else                         skippedComps.push_back(comp);
    }

    vector<double> distEnum;
    distEnum.push_back(1.0);
    int maxEnumMines = 0;
    for ( int comp : enumeratedComps )
    {
        int nc = (int)componentCells[comp].size();
        vector<double> conv(distEnum.size() + nc, 0.0);
        for ( size_t i = 0; i < distEnum.size(); ++i )
        {
            if ( distEnum[i] == 0.0 ) continue;
            for ( int k = 0; k <= nc; ++k )
            {
                double w = compWaysAt[comp][k];
                if ( w == 0.0 ) continue;
                conv[i + k] += distEnum[i] * w;
            }
        }
        distEnum = move(conv);
        maxEnumMines += nc;
    }

    double totalWeight = 0.0;
    vector<double> weightAt(distEnum.size(), 0.0);
    for ( size_t k = 0; k < distEnum.size(); ++k )
    {
        if ( distEnum[k] == 0.0 ) continue;
        int leftForRest = minesLeft - (int)k;
        int restCells = nonFrontierCovered;
        for ( int comp : skippedComps ) restCells += (int)componentCells[comp].size();
        if ( leftForRest < 0 || leftForRest > restCells ) continue;
        double w = binomial(restCells, leftForRest);
        weightAt[k] = distEnum[k] * w;
        totalWeight += weightAt[k];
    }

    if ( totalWeight == 0.0 )
    {
        return { LEAVE, -1, -1 };
    }

    vector<vector<double>> cellMineProb(nComp);
    for ( int comp = 0; comp < nComp; ++comp )
        cellMineProb[comp].assign(componentCells[comp].size(), 0.0);

    for ( int comp : enumeratedComps )
    {
        int nc = (int)componentCells[comp].size();

        vector<double> distOther = {1.0};
        for ( int other : enumeratedComps )
        {
            if ( other == comp ) continue;
            int nco = (int)componentCells[other].size();
            vector<double> conv(distOther.size() + nco, 0.0);
            for ( size_t i = 0; i < distOther.size(); ++i )
            {
                if ( distOther[i] == 0.0 ) continue;
                for ( int k = 0; k <= nco; ++k )
                {
                    double w = compWaysAt[other][k];
                    if ( w == 0.0 ) continue;
                    conv[i + k] += distOther[i] * w;
                }
            }
            distOther = move(conv);
        }

        int restCells = nonFrontierCovered;
        for ( int sk : skippedComps ) restCells += (int)componentCells[sk].size();

        for ( int j = 0; j <= nc; ++j )
        {
            double waysThisJ = compWaysAt[comp][j];
            if ( waysThisJ == 0.0 ) continue;

            double weightJ = 0.0;
            for ( size_t ko = 0; ko < distOther.size(); ++ko )
            {
                if ( distOther[ko] == 0.0 ) continue;
                int needRest = minesLeft - j - (int)ko;
                if ( needRest < 0 || needRest > restCells ) continue;
                weightJ += distOther[ko] * binomial(restCells, needRest);
            }
            if ( weightJ == 0.0 ) continue;

            for ( int ii = 0; ii < nc; ++ii )
            {
                double cellHits = compCellMineAt[comp][j][ii];
                if ( cellHits == 0.0 ) continue;
                cellMineProb[comp][ii] += (cellHits / waysThisJ) * weightJ;
            }
        }
    }

    for ( int comp : enumeratedComps )
    {
        for ( double& p : cellMineProb[comp] )
            p /= totalWeight;
    }

    double nonFrontierP = 0.5;
    {
        int restCells = nonFrontierCovered;
        for ( int sk : skippedComps ) restCells += (int)componentCells[sk].size();
        if ( restCells > 0 )
        {
            double sumP = 0.0;
            for ( size_t k = 0; k < weightAt.size(); ++k )
            {
                if ( weightAt[k] == 0.0 ) continue;
                int rem = minesLeft - (int)k;
                if ( rem < 0 ) rem = 0;
                if ( rem > restCells ) rem = restCells;
                sumP += weightAt[k] * ((double)rem / (double)restCells);
            }
            nonFrontierP = sumP / totalWeight;
            if ( nonFrontierP < 0.0 ) nonFrontierP = 0.0;
            if ( nonFrontierP > 1.0 ) nonFrontierP = 1.0;
        }
    }

    bool queued = false;
    for ( int comp : enumeratedComps )
    {
        int nc = (int)componentCells[comp].size();
        for ( int ii = 0; ii < nc; ++ii )
        {
            double p = cellMineProb[comp][ii];
            int localIdx = componentCells[comp][ii];
            int id       = frontierCells[localIdx];
            auto xy = idToXY(id);
            if ( p < 1e-9 )
            {
                if ( state[xy.first][xy.second] == COVERED && !inSafeQ[xy.first][xy.second] )
                {
                    safeQueue.emplace_back(xy.first, xy.second);
                    inSafeQ[xy.first][xy.second] = true;
                    queued = true;
                }
            }
            else if ( p > 1.0 - 1e-9 )
            {
                if ( state[xy.first][xy.second] == COVERED && !inMineQ[xy.first][xy.second] )
                {
                    mineQueue.emplace_back(xy.first, xy.second);
                    inMineQ[xy.first][xy.second] = true;
                    queued = true;
                }
            }
        }
    }

    if ( queued )
    {
        Action a = nextPlannedAction();
        if ( a.action != LEAVE ) return a;
    }

    struct Candidate { double p; int id; int coveredNbrs; int uncoveredNumNbrs; };
    vector<Candidate> cands;
    cands.reserve(totalCells);

    for ( int comp : enumeratedComps )
    {
        int nc = (int)componentCells[comp].size();
        for ( int ii = 0; ii < nc; ++ii )
        {
            int localIdx = componentCells[comp][ii];
            int id       = frontierCells[localIdx];
            cands.push_back({ cellMineProb[comp][ii], id, 0, 0 });
        }
    }

    {
        vector<double> localProb(nFrontier, -1.0);
        for ( const auto& lc : lcs )
        {
            if ( lc.cells.empty() ) continue;
            double pp = (double)lc.mines / (double)lc.cells.size();
            for ( int loc : lc.cells )
                if ( pp > localProb[loc] ) localProb[loc] = pp;
        }
        for ( int comp : skippedComps )
        {
            for ( int loc : componentCells[comp] )
            {
                int id = frontierCells[loc];
                double pp = localProb[loc];
                if ( pp < 0.0 ) pp = nonFrontierP;
                cands.push_back({ pp, id, 0, 0 });
            }
        }
    }

    for ( int x = 0; x < colDim; ++x )
    {
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[x][y] != COVERED ) continue;
            int id = cellId(x, y);
            if ( cellToLocal[id] != -1 ) continue;
            cands.push_back({ nonFrontierP, id, 0, 0 });
        }
    }

    if ( cands.empty() ) return { LEAVE, -1, -1 };

    vector<pair<int,int>> nbrs;
    nbrs.reserve(8);
    for ( auto& cd : cands )
    {
        auto xy = idToXY(cd.id);
        const_cast<MyAI*>(this)->getNeighbors(xy.first, xy.second, nbrs);
        int cov = 0, unc = 0;
        for ( auto& p : nbrs )
        {
            CellState s = state[p.first][p.second];
            if      ( s == COVERED   ) ++cov;
            else if ( s == UNCOVERED ) ++unc;
        }
        cd.coveredNbrs = cov;
        cd.uncoveredNumNbrs = unc;
    }

    int bestI = 0;
    for ( int i = 1; i < (int)cands.size(); ++i )
    {
        const Candidate& A = cands[bestI];
        const Candidate& B = cands[i];
        if ( B.p < A.p - 1e-9 ) { bestI = i; continue; }
        if ( B.p > A.p + 1e-9 ) continue;
        if ( B.coveredNbrs > A.coveredNbrs ) { bestI = i; continue; }
        if ( B.coveredNbrs < A.coveredNbrs ) continue;
        if ( B.uncoveredNumNbrs > A.uncoveredNumNbrs ) { bestI = i; }
    }

    auto bestXY = idToXY(cands[bestI].id);
    return { UNCOVER, bestXY.first, bestXY.second };
}

Agent::Action MyAI::localProbabilityGuess ( )
{
    int coveredRemaining = 0;
    for ( int x = 0; x < colDim; ++x )
        for ( int y = 0; y < rowDim; ++y )
            if ( state[x][y] == COVERED ) ++coveredRemaining;

    if ( coveredRemaining == 0 ) return { LEAVE, -1, -1 };

    int minesLeft = totalMines - flaggedCount;
    if ( minesLeft < 0 ) minesLeft = 0;

    vector<Constraint> cs;
    buildConstraints(cs);

    if ( cs.empty() )
    {
        int corners[4][2] = { {0,0}, {colDim-1,0}, {0,rowDim-1}, {colDim-1,rowDim-1} };
        for ( int i = 0; i < 4; ++i )
        {
            int x = corners[i][0], y = corners[i][1];
            if ( state[x][y] == COVERED ) return { UNCOVER, x, y };
        }
        for ( int x = 0; x < colDim; ++x )
        {
            if ( state[x][0]         == COVERED ) return { UNCOVER, x, 0 };
            if ( state[x][rowDim-1]  == COVERED ) return { UNCOVER, x, rowDim - 1 };
        }
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[0][y]         == COVERED ) return { UNCOVER, 0, y };
            if ( state[colDim-1][y]  == COVERED ) return { UNCOVER, colDim - 1, y };
        }
        for ( int x = 0; x < colDim; ++x )
            for ( int y = 0; y < rowDim; ++y )
                if ( state[x][y] == COVERED ) return { UNCOVER, x, y };
        return { LEAVE, -1, -1 };
    }

    vector<vector<double>> prob(colDim, vector<double>(rowDim, -1.0));
    vector<vector<bool>>   onFrontier(colDim, vector<bool>(rowDim, false));

    for ( const auto& c : cs )
    {
        if ( c.cells.empty() ) continue;
        double local = (double)c.mines / (double)c.cells.size();
        for ( int id : c.cells )
        {
            auto p = idToXY(id);
            onFrontier[p.first][p.second] = true;
            if ( local > prob[p.first][p.second] )
                prob[p.first][p.second] = local;
        }
    }

    int nonFrontierCovered = 0;
    double frontierMineEst = 0.0;
    for ( int x = 0; x < colDim; ++x )
    {
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[x][y] == COVERED && !onFrontier[x][y] ) ++nonFrontierCovered;
            if ( onFrontier[x][y] ) frontierMineEst += prob[x][y];
        }
    }

    double nonFrontierP = 1.0;
    if ( nonFrontierCovered > 0 )
    {
        double rem = (double)minesLeft - frontierMineEst;
        if ( rem < 0.0 ) rem = 0.0;
        nonFrontierP = rem / (double)nonFrontierCovered;
        if ( nonFrontierP < 0.0 ) nonFrontierP = 0.0;
        if ( nonFrontierP > 1.0 ) nonFrontierP = 1.0;
    }

    int bestX = -1, bestY = -1;
    double bestP = 2.0;
    int    bestCov = -1;

    for ( int x = 0; x < colDim; ++x )
    {
        for ( int y = 0; y < rowDim; ++y )
        {
            if ( state[x][y] != COVERED ) continue;
            double p = onFrontier[x][y] ? prob[x][y] : nonFrontierP;

            int cov = 0;
            for ( int dx = -1; dx <= 1; ++dx )
                for ( int dy = -1; dy <= 1; ++dy )
                    if ( !(dx == 0 && dy == 0) && inBounds(x+dx, y+dy) && state[x+dx][y+dy] == COVERED )
                        ++cov;

            if ( p < bestP - 1e-9 || (p < bestP + 1e-9 && cov > bestCov) )
            {
                bestP = p; bestCov = cov;
                bestX = x; bestY = y;
            }
        }
    }

    if ( bestX < 0 ) return { LEAVE, -1, -1 };
    return { UNCOVER, bestX, bestY };
}
// ======================================================================
// YOUR CODE ENDS
// ======================================================================