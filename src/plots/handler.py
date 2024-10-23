import json
from extract_data import extract_query_data
from hit_miss_rate import plot_hit_miss_rate
from hit_miss_distr import plot_hit_miss_expired_distribution
from hit_miss_rolling_avg import plot_hit_miss_expired_rolling_avg
from rate_per_balloon import plot_average_rate_per_balloon
from heatmap_plot import plot_heatmap_cache_status

def compute_plot(cache_type):
    path = f"/home/intou/Desktop/IoT/IoT-Project-2024/src/output_tests/s6_b4_534/{cache_type}.json"
    with open(path) as f:
        data = json.load(f)

    balloon_responses, query_count = extract_query_data(data)

    plot_hit_miss_expired_distribution(data, balloon_responses, query_count)
    # plot_hit_miss_rate(data, balloon_responses, query_count)
    # plot_hit_miss_expired_rolling_avg(data, balloon_responses, query_count)
    # plot_average_rate_per_balloon(data, balloon_responses)
    plot_heatmap_cache_status(data, balloon_responses, query_count)


if __name__ == "__main__":

    for c in ["FIFO", "LFU", "LRU"]:
        compute_plot(c)