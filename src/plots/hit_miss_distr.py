import matplotlib.pyplot as plt
import numpy as np
import os

def plot_hit_miss_expired_distribution(data: dict, balloon_responses: dict, query_count: int):
    plt.figure(figsize=(15, 8))

    hit_counts = []
    miss_counts = []
    expired_counts = []

    balloons = []
    for balloon, responses in balloon_responses.items():
        hits = responses.count(1)
        misses = responses.count(0)
        expired = responses.count(-1)
        
        hit_counts.append(hits)
        miss_counts.append(misses)
        expired_counts.append(expired)
        balloons.append(balloon)

    bar_width = 0.25
    indices = np.arange(len(balloons))

    plt.bar(indices, hit_counts, bar_width, label='Hits', color='green')
    plt.bar(indices + bar_width, miss_counts, bar_width, label='Misses', color='blue')
    plt.bar(indices + 2 * bar_width, expired_counts, bar_width, label='Expired', color='red')

    plt.xlabel('Balloons', fontsize=14)
    plt.ylabel('Count', fontsize=14)
    plt.title(f'Cache Hit/Miss/Expired Distribution per Balloon (tot_queries={query_count})', fontsize=16, fontweight='bold')
    
    metadata_text = f"Sensors: {data['sensors']} | Cache Type: {data['cache_type']} | Cache Expiration: {data['cache_expiration']} | Cache Size: {data['cache_size']}"
    # plt.suptitle(metadata_text, fontsize=12)

    plt.xticks(indices + bar_width, balloons)

    plt.legend()
    plt.grid(True)

    plot_type = "hit_miss_distribution"
    base_path = f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/"
    folder = f"s{data['sensors']}_b{data['balloons']}_{data['cache_expiration']}{data['cache_size']}{data['query_rate']}"
    if not os.path.exists(base_path + folder):
        os.makedirs(base_path + folder)

    plt.savefig(f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/{folder}/{data['cache_type']}.pdf", format='pdf')
    plt.tight_layout()
    plt.show()
