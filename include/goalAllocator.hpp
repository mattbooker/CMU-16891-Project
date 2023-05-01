#include <iostream>
#include <algorithm>
#include <vector>

class goalAllocator
{
    public:
    goalAllocator(std::vector<std::vector<int>> costMatrix, int numOfRows, int numOfCols)
    {
        costMatrix_ = costMatrix;
        numOfCols = numOfCols_;
        numOfRows = numOfRows_;
    };
    std::vector<std::vector<int>> solve();

    private:

    void rowMinimization(int& step);
    void columnMinimization(int& step);
    void step2(int& step);
    void step3(int& step);
    void step4(int path_row_0, int path_col_0, int& step);
    void step5(std::vector<std::vector<int>> path, int path_row_0, int path_col_0, int& step);
    void step6(int& step);
    void clear_covers(std::vector<int>& cover);
    void find_star_in_col(int c, 
                        int& r);
    void find_prime_in_row(int r, 
                           int& c);
    void augment_path(std::vector<std::vector<int>>& path, 
                        int path_count);
    void erase_primes();
    void find_smallest(int& minval);
    void find_a_zero(int& row, 
                    int& col);
    bool star_in_row(int row);
    void find_star_in_row(int row,
                        int& col);
    // void find_smallest(int& minval);
    

    std::vector<std::vector<int>> costMatrix_;    
    std::vector<std::vector<int>> maskMatrix_;   
    std::vector<int> RowCover;
    std::vector<int> ColCover; 
    int numOfRows_, numOfCols_;

};