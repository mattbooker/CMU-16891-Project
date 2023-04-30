#include <iostream>
#include <algorithm>
#include <vector>

class goalAllocator
{
    goalAllocator();

    void solve();

    void rowMinimization(int& step);
    void columnMinimization(int& step);
    void step2(int& step);
    void step3(int& step);
    void step4(int& step);
    void step5(int& step);
    void clear_covers(std::vector<int>& cover);
    void find_star_in_col(int c, 
                        int& r);
    void find_prime_in_row(int r, 
                           int& c);
    void augment_path(std::vector<std::vector<int>>& path, 
                        int path_count);
    void erase_primes();
    void find_smallest(float& minval);
    void find_a_zero(int& row, 
                    int& col);
    bool star_in_row(int row);
    void find_star_in_row(int row,
                        int& col);
    

    std::vector<std::vector<float>> costMatrix_;    
    std::vector<std::vector<int>> maskMatrix_;   
    std::vector<int> RowCover;
    std::vector<int> ColCover; 
    int numOfRows, numOfCols;

};