#include <iostream>
#include <cstdint>
using namespace std;


uint64_t addrGen(unsigned chan, unsigned rank, unsigned bankgroup, unsigned bank,
                                 unsigned row, unsigned col)
{
    uint64_t addr = 0;
    if (1)
    {
        addr = rank;

        addr <<= num_row_bits_;
        addr |= row;

        addr <<= num_col_bits_;
        addr |= col;

        addr <<= num_bankgroup_bits_;
        addr |= bankgroup;

        addr <<= num_bank_bits_;
        addr |= bank;

        addr <<= num_chan_bits_;
        addr |= chan;

        addr <<= num_offset_bits_;
    }
    else
    {
        cerr << "Fatal: Not supported address scheme for PIM controller" << endl;
    }
    return addr;
}