import seaborn as sns
import numpy as np
import matplotlib.pylab as plt
import os

def plot_heatmap_cache_status(data: dict, balloon_responses: dict, query_count: int):
    plt.figure(figsize=(15, 8))

    balloons = list(balloon_responses.keys())
    heatmap_data = np.array([balloon_responses[balloon] for balloon in balloons])

    sns.heatmap(heatmap_data, cmap='RdYlGn', cbar_kws={'label': 'Cache Status (-1: Expired, 0: Miss, 1: Hit)'}, 
                yticklabels=balloons, xticklabels=False)

    plt.xlabel('Query Index', fontsize=14)
    plt.ylabel('Balloons', fontsize=14)
    plt.title(f'Heatmap of Cache Hits/Misses/Expired Over Time (tot_queries={query_count})', fontsize=16, fontweight='bold')
    
    metadata_text = f"Sensors: {data['sensors']} | Cache Type: {data['cache_type']} | Cache Expiration: {data['cache_expiration']} | Cache Size: {data['cache_size']}"
    # plt.suptitle(metadata_text, fontsize=12)

    plot_type = "heatmap"
    base_path = f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/"
    folder = f"s{data['sensors']}_b{data['balloons']}_{data['cache_expiration']}{data['cache_size']}{data['query_rate']}"
    if not os.path.exists(base_path + folder):
        os.makedirs(base_path + folder)

    plt.savefig(f"/home/intou/Desktop/IoT/IoT-Project-2024/images/{plot_type}/{folder}/{data['cache_type']}.pdf", format='pdf')
    
    plt.tight_layout()
    plt.show()
