#ifndef SQUARE_DETECTOR_HH
#define SQUARE_DETECTOR_HH
#include <cmath>

template<
    int detect_period,
    int period_count = 16,
    int patience_multiplier = 1,
    int correlation_cutoff_percentage = 33
>
class square_detector
{
public:
    square_detector()
    :   correlation_x{0},
        correlation_y{0},
        correlation_x_sum(0),
        correlation_y_sum(0),
        correlation_head(0),
        phase_counter(0),
        patience_counter(0)
    {
        int phase = 0;

        for(int i = 0; i < phase_table_size; ++i)
        {
            phase_table[i] = 0;
            for(int j = 0; j < 32; ++j)
            {
                int bit = ((phase/half_detect_period)&1);
                phase_table[i] |= bit << j;
                phase += 1;
            }
        }

    }

    void push_buffer(short* buffer, int samples)
    {
        for(int i = 0; i < samples;)
        {
            int w = 0;
            for(int j = 0; j < bits_per_word; ++j, ++i)
                w |= buffer[i] < 0 ? 1<<j : 0;
            push_word(w);
        }
    }

    bool detection() const
    {
        return patience_counter > 0;
    }

private:
    static const int half_detect_period = detect_period/2;
    static const int history_bits = detect_period * period_count;
    static const int bits_per_word = sizeof(int)*8;
    static const int history_words = history_bits/bits_per_word;
    static const int patience = detect_period * patience_multiplier;
    static const int correlation_cutoff =
        history_bits * correlation_cutoff_percentage / 200;
    static const int phase_table_size =
        (half_detect_period * 2 + bits_per_word - 1)/bits_per_word + 1;

    // Could just as well be char if storage is an issue! The values are
    // between -16 and 16.
    int correlation_x[history_words];
    int correlation_y[history_words];
    int correlation_x_sum;
    int correlation_y_sum;
    int correlation_head;
    unsigned phase_table[phase_table_size];
    unsigned phase_counter;
    int patience_counter;

    unsigned get_phase_mask(unsigned phase) const
    {
        unsigned low_bit_index = phase % (half_detect_period*2);
        unsigned high_bit_index = low_bit_index + 31;
        unsigned low = low_bit_index >> 5;
        unsigned high = high_bit_index >> 5;
        unsigned low_offset = low_bit_index & 0x1F;
        unsigned high_offset = (~high_bit_index) & 0x1F;

        unsigned mask = (phase_table[low] >> low_offset) |
            (phase_table[high] << high_offset);

        return mask;
    }

    void push_word(int w)
    {
        int x = get_phase_mask(phase_counter);
        int y = get_phase_mask(phase_counter + (half_detect_period>>1));
        phase_counter += bits_per_word;

        int sx = __builtin_popcount(x ^ w) - 16;
        int sy = __builtin_popcount(y ^ w) - 16;

        correlation_x_sum += sx - correlation_x[correlation_head];
        correlation_y_sum += sy - correlation_y[correlation_head];

        correlation_x[correlation_head] = sx;
        correlation_y[correlation_head] = sy;
        correlation_head = (correlation_head+1) % history_words;

        int cx = abs(correlation_x_sum);
        int cy = abs(correlation_y_sum);

        if(cx + cy > correlation_cutoff)
            patience_counter = patience;

        patience_counter -= 1;
    }
};
#endif
