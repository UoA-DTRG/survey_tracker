#include <vector>

using namespace std;

template <typename T>
class RingBuffer{
private:
    vector<T> _dat;
    int _size;
    int _write_ind;

public:
    RingBuffer(int size){
        _size = size;
        _dat.resize(size);
        _write_ind = 0;
    }

    bool enqueue(T sample){
        _dat[_write_ind] = sample;
        _write_ind++;
        _write_ind %= _size;

        return true;
    }

    T get_rms_sqr(){
        T ret{0};
        for(int i = 0; i < _size; ++i){
            ret += _dat[i] * _dat[i];
        }
        return ret / (double) _size;
    }
};