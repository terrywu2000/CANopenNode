#ifndef XNET_CO_TIMED_HISTORY_HPP
#define XNET_CO_TIMED_HISTORY_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace xco {

/*
 * The caller supplies one validated clock domain and starts a new epoch when
 * the mapping becomes invalid. available_time is when the complete record was
 * first committed and accepted. All times use the same integer unit.
 * The watermark promises that no more records at or before that source time
 * can arrive; an empty software queue does not establish such a promise.
 */
enum class TimedHistoryResult {
    ok, no_candidate, not_closed, deadline_missed, history_gap, stale_epoch, invalid, overflow, invalidated
};

template<class T, std::size_t Capacity> class TimedHistory {
    static_assert(Capacity > 0, "history must have a positive capacity");
public:
    struct Record {
        uint64_t epoch;
        uint64_t sequence;
        uint64_t source_time;
        uint64_t available_time;
        T value;
    };

    TimedHistoryResult start_epoch(uint64_t epoch, std::size_t capacity = Capacity, uint64_t min_spacing = 0) {
        if (epoch == 0 || epoch <= epoch_) return TimedHistoryResult::stale_epoch;
        if (capacity == 0 || capacity > Capacity) return TimedHistoryResult::invalid;
        epoch_ = epoch;
        count_ = 0;
        capacity_ = capacity;
        min_spacing_ = min_spacing;
        last_source_.reset();
        last_sequence_ = 0;
        closed_.reset();
        last_observed_at_.reset();
        retired_.reset();
        valid_ = true;
        return TimedHistoryResult::ok;
    }

    void invalidate() { valid_ = false; }

    TimedHistoryResult append(const Record& record) {
        if (!valid_) return TimedHistoryResult::invalidated;
        if (record.epoch != epoch_) return TimedHistoryResult::stale_epoch;
        if (record.sequence == 0 || record.sequence <= last_sequence_) {
            valid_ = false;
            return TimedHistoryResult::invalidated;
        }
        if (closed_ && record.source_time <= *closed_) {
            valid_ = false;
            return TimedHistoryResult::invalidated;
        }
        if (min_spacing_ && last_source_
            && (record.source_time < *last_source_ || record.source_time - *last_source_ < min_spacing_)) {
            valid_ = false;
            return TimedHistoryResult::invalidated;
        }
        if (last_observed_at_ && record.available_time < *last_observed_at_) {
            valid_ = false;
            return TimedHistoryResult::invalidated;
        }
        if (count_ == capacity_) {
            valid_ = false;
            return TimedHistoryResult::overflow;
        }
        records_[count_++] = record;
        last_sequence_ = record.sequence;
        last_source_ = record.source_time;
        last_observed_at_ = record.available_time;
        return TimedHistoryResult::ok;
    }

    TimedHistoryResult close_through(uint64_t epoch, uint64_t watermark, uint64_t observed_at) {
        if (!valid_) return TimedHistoryResult::invalidated;
        if (epoch != epoch_) return TimedHistoryResult::stale_epoch;
        if ((closed_ && watermark < *closed_) || (last_observed_at_ && observed_at < *last_observed_at_))
            return TimedHistoryResult::invalid;
        closed_ = watermark;
        last_observed_at_ = observed_at;
        return TimedHistoryResult::ok;
    }

    /* A later selection reports a gap if its age window intersects retirement. */
    TimedHistoryResult retire_through(uint64_t epoch, uint64_t time) {
        if (!valid_) return TimedHistoryResult::invalidated;
        if (epoch != epoch_) return TimedHistoryResult::stale_epoch;
        if (!closed_ || time > *closed_ || (retired_ && time < *retired_))
            return TimedHistoryResult::invalid;
        std::size_t kept = 0;
        for (std::size_t i = 0; i < count_; ++i)
            if (records_[i].source_time > time) records_[kept++] = records_[i];
        count_ = kept;
        retired_ = time;
        return TimedHistoryResult::ok;
    }

    TimedHistoryResult retire_before(uint64_t epoch, uint64_t oldest) {
        if (!valid_) return TimedHistoryResult::invalidated;
        if (epoch != epoch_) return TimedHistoryResult::stale_epoch;
        if (oldest == 0) return TimedHistoryResult::ok;
        return retire_through(epoch, oldest - 1);
    }

    TimedHistoryResult select(uint64_t epoch, uint64_t cutoff, uint64_t epsilon,
                              uint64_t decision_time, uint64_t deadline,
                              uint64_t max_age, Record& result) const {
        if (!valid_) return TimedHistoryResult::invalidated;
        if (epoch != epoch_) return TimedHistoryResult::stale_epoch;
        if (decision_time >= deadline) return TimedHistoryResult::deadline_missed;
        if (last_observed_at_ && decision_time < *last_observed_at_) return TimedHistoryResult::invalid;
        if (!closed_ || *closed_ < cutoff) return TimedHistoryResult::not_closed;
        const uint64_t oldest = cutoff > max_age ? cutoff - max_age : 0;
        if (retired_ && oldest <= *retired_) return TimedHistoryResult::history_gap;
        if (epsilon > cutoff) return TimedHistoryResult::no_candidate;
        const uint64_t latest = cutoff - epsilon;
        const Record* selected = nullptr;
        for (std::size_t i = 0; i < count_; ++i) {
            const auto& record = records_[i];
            if (record.source_time < oldest || record.source_time > latest) continue;
            if (record.available_time > decision_time) return TimedHistoryResult::invalid;
            if (!selected || record.source_time > selected->source_time
                || (record.source_time == selected->source_time && record.sequence > selected->sequence))
                selected = &record;
        }
        if (!selected) return TimedHistoryResult::no_candidate;
        result = *selected;
        return TimedHistoryResult::ok;
    }

private:
    std::array<Record, Capacity> records_{};
    std::size_t count_ = 0;
    std::size_t capacity_ = Capacity;
    uint64_t min_spacing_ = 0;
    uint64_t epoch_ = 0, last_sequence_ = 0;
    std::optional<uint64_t> closed_, retired_, last_observed_at_, last_source_;
    bool valid_ = false;
};

} // namespace xco

#endif
