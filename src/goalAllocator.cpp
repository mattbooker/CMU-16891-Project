#include "goalAllocator.hpp"

void goalAllocator::solve()
{
    int numOfRows;
    int numOfCols;
    
    int path_row_0, path_col_0; //temporary to hold the smallest uncovered value
    
    // Array for the augmenting path algorithm
    std::vector<std::vector<int>> path (costMatrix_.size()+1, std::vector<int>(2, 0));
    
    /* Now Work The Steps */
    bool done = false;
    int step = 1;
    while (!done) {
        switch (step) {
            case 1:
                rowMinimization(step);
                columnMinimization(step);
                break;
            case 2:
                step2(step);
                break;
            case 3:
                step3(step);
                break;
            case 4:
                step4(path_row_0, path_col_0, step);
                break;
            case 5:
                step5(path, path_row_0, path_col_0, step);
                break;
            case 6:
                step6(step);
                break;
            case 7:
                for (auto& vec: M) {vec.resize(original.begin()->size());}
                M.resize(original.size());
                done = true;
                break;
            default:
                done = true;
                break;
        }
    }
}

void goalAllocator::rowMinimization()
{
    for (auto &row: costMatrix_)
    {
        // auto min_element = *std::min_element(begin(row), end(row));
        auto min_element = *std::min_element(row.begin(), row.end());
        if (min_element > 0)
        {
            for(auto &element: row)
            {
                element -= min_element;
            }
        }
    }
}

void goalAllocator::columnMinimization()
{
    for (int i = 0; i < numOfRows; i++)
    {
        int minm = costMatrix_[0][i];
        for (int j = 0; j < numOfCols; j++)
        {
            if (costMatrix_[j][i] < minm)
                minm = costMatrix_[j][i];
        }

        for (int j = 0; j < numOfCols; j++)
        {
            costMatrix_[j][i] -= minm;
        }
    }

}

inline void goalAllocator::clear_covers(std::vector<int>& cover) 
{
    for (auto& n: cover) n = 0;
}

void goalAllocator::step2()
{
    for (int r = 0; r < numOfRows; r++)
    {
        for (int c = 0; c < numOfCols; c++)
        {
            if (costMatrix_[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) 
            {
                maskMatrix_[r][c] = 1;
                RowCover[r] = 1;
                ColCover[c] = 1;
            }
            
            clear_covers(RowCover);
            clear_covers(ColCover);
        }
    }
}

void goalAllocator::step3()
{
    int colCount;
    for (int i = 0; i < numOfRows; i++)
    {
        for (int j = 0; j < numOfCols; j++)
        {
            if (maskMatrix_[i][j] == 1)
                ColCover[j] = 1;
        }
    }
    colCount = 0;
    for (int j = 0; j < numOfCols; j++)
    {
        if (ColCover[j] == 1)
            colCount++;
    }
    if (colCount >= numOfCols || colCount >= numOfRows)
        //step 7
    else
        //step 4
}

void goalAllocator::find_a_zero(int& row, int& col)
{
    int r = 0;
    int c = 0;
    int sz = costMatrix_.size();
    bool done = false;
    row = -1;
    col = -1;
    
    while (!done) {
        c = 0;
        while (true) {
            if (maskMatrix_[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) {
                row = r;
                col = c;
                done = true;
            }
            c += 1;
            if (c >= sz || done)
                break;
        }
        r += 1;
        if (r >= sz)
            done = true;
    }
}

bool goalAllocator::star_in_row(int row)
{
    bool tmp = false;
    for (unsigned c = 0; c < maskMatrix_.size(); c++)
        if (maskMatrix_[row][c] == 1)
            tmp = true;
    
    return tmp;
}


void goalAllocator::find_star_in_row(int row,
                      int& col)
{
    col = -1;
    for (unsigned c = 0; c < maskMatrix_.size(); c++)
        if (maskMatrix_[row][c] == 1)
            col = c;
}

void goalAllocator::step4()
{
    int row = -1;
    int col = -1;
    bool done;

    done = false;
    while(!done)
    {
        find_a_zero(row, col);
        if (row == -1)
        {
            done = true;
            step = 6;
        }
        else{
            maskMatrix_[row][col] = 2;
            if (star_in_row(row))
            {
                find_star_in_row(row, col);
                RowCover[row] = 1;
                ColCover[col] = 0;
            }
            else{
                done = true;
                step = 5;
                path_row_0 = row;
                path_col_0 = col;
    
            }
        }
    }

}

void goalAllocator::find_star_in_col(int c, 
                      int& r)
{
    r = -1;
    for (unsigned i = 0; i < maskMatrix_.size(); i++)
        if (maskMatrix_[i][c] == 1)
            r = i;
}

void goalAllocator::find_prime_in_row(int r, 
                       int& c)
{
    for (unsigned j = 0; j < maskMatrix_.size(); j++)
        if (maskMatrix_[r][j] == 2)
            c = j;
}

void goalAllocator::augment_path(std::vector<std::vector<int>>& path, 
                  int path_count)
{
    for (int p = 0; p < path_count; p++)
        if (maskMatrix_[path[p][0]][path[p][1]] == 1)
            maskMatrix_[path[p][0]][path[p][1]] = 0;
        else
            maskMatrix_[path[p][0]][path[p][1]] = 1;
}

void goalAllocator::erase_primes()
{
    for (auto& row: maskMatrix_)
        for (auto& val: row)
            if (val == 2)
                val = 0;
}

void goalAllocator::step5(std::vector<std::vector<int>>& path, 
           int path_row_0, 
           int path_col_0,
           int& step)
{
    int r = -1;
    int c = -1;
    int path_count = 1;
    
    path[path_count - 1][0] = path_row_0;
    path[path_count - 1][1] = path_col_0;
    
    bool done = false;
    while (!done) {
        find_star_in_col(path[path_count - 1][1], r);
        if (r > -1) {
            path_count += 1;
            path[path_count - 1][0] = r;
            path[path_count - 1][1] = path[path_count - 2][1];
        }
        else {done = true;}
        
        if (!done) {
            find_prime_in_row(path[path_count - 1][0], c);
            path_count += 1;
            path[path_count - 1][0] = path[path_count - 2][0];
            path[path_count - 1][1] = c;
        }
    }
    
    augment_path(path, path_count);
    clear_covers(RowCover);
    clear_covers(ColCover);
    erase_primes();
    
    step = 3;
}

void goalAllocator::find_smallest(float& minval)
{
    for (unsigned r = 0; r < costMatrix_.size(); r++)
        for (unsigned c = 0; c < costMatrix_.size(); c++)
            if (RowCover[r] == 0 && ColCover[c] == 0)
                if (minval > costMatrix_[r][c])
                    minval = costMatrix_[r][c];
}

void step6(int& step)
{
    float minval = std::numeric_limits<float>::max();
    find_smallest(minval);
    
    int sz = costMatrix_.size();
    for (int r = 0; r < sz; r++)
        for (int c = 0; c < sz; c++) {
            if (RowCover[r] == 1)
                costMatrix_[r][c] += minval;
            if (ColCover[c] == 0)
                costMatrix_[r][c] -= minval;
    }
    
    step = 4;
}