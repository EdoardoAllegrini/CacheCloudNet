import matplotlib.pyplot as plt
import numpy as np
import json
import os

def plot_hit_miss_rate(data: dict, balloon_responses: dict, query_count: int):
    query_interval = round(data["queries"][1]["timestamp_request_start"]-data["queries"][0]["timestamp_request_start"])
    
    plt.figure(figsize=(20, 10))
    time_intervals = np.arange(0, query_interval*query_count, query_interval)

    
    markers = ['o', 's', '^', 'd', 'x', '*', 'p', 'h', 'v']  
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'orange', 'purple']
    
    for idx, (balloon, responses) in enumerate(balloon_responses.items()):
        plt.plot(time_intervals, responses, marker=markers[idx % len(markers)], 
        label=balloon, linestyle='-', color=colors[idx % len(colors)])

    
    metadata_text = f"Sensors: {data['sensors']} | Cache Type: {data['cache_type']} | Cache Expiration: {data['cache_expiration']} | Cache Size: {data['cache_size']}"
    plt.suptitle(metadata_text, fontsize=12)  

    
    plt.title(f'Hit/Miss/Expired Cache Rate Over Time (tot_queries={query_count})', fontsize=16, fontweight='bold')
    plt.xlabel('Query Time', fontsize=14)
    plt.ylabel('Status', fontsize=14)
    plt.grid(True)
    plt.legend(loc='upper right', fontsize=12)

    
    plt.yticks([-1, 0, 1], ['Expired', 'Miss', 'Hit'])

    plt.xticks(time_intervals)

    plot_type = "hit_miss_rate"
    base_path = f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/"
    folder = f"s{data['sensors']}_b{data['balloons']}_{data['cache_expiration']}{data['cache_size']}{data['query_rate']}"
    if not os.path.exists(base_path + folder):
        os.makedirs(base_path + folder)

    plt.savefig(f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/{folder}/{data['cache_type']}.pdf", format='pdf')

    plt.tight_layout()
    plt.show()



