import os
import json
import numpy as np
import matplotlib.pyplot as plt

START_IDX = 5
END_IDX = 75



def parse_cache_logs(file_path):
    hit_count = 0
    miss_count = 0
    expiration_count = 0
    total_queries = 0

    
    with open(file_path, 'r') as f:
        data = json.load(f)
        for query in data["queries"][START_IDX:END_IDX]:
            total_queries += 1

            
            for balloon, details in query.items():
                if isinstance(details, dict) and 'data' in details:
                    if details['data'] == 'KeyNotFoundError':
                        miss_count += 1
                    elif details['data'] == 'CacheExpiredError':
                        expiration_count += 1
                    else:
                        hit_count += 1

    return hit_count, miss_count, expiration_count, total_queries


def compute_metrics(file_path):
    hit_count, miss_count, expiration_count, total_queries = parse_cache_logs(file_path)

    
    hit_rate = hit_count / total_queries if total_queries > 0 else 0
    miss_rate = miss_count / total_queries if total_queries > 0 else 0
    expiration_rate = expiration_count / total_queries if total_queries > 0 else 0

    return {
        "total_queries": total_queries,
        "hit_rate": hit_rate,
        "miss_rate": miss_rate,
        "expiration_rate": expiration_rate,
    }


def main():
    base_folder = "/home/intou/Desktop/IoT/IoT-Project-2024/src/output_tests"  
    cache_sizes = [3, 4, 5]
    cache_types = ['FIFO', 'LRU', 'LFU']

    
    hit_rates = np.zeros((len(cache_types), len(cache_sizes)))
    miss_rates = np.zeros((len(cache_types), len(cache_sizes)))
    expiration_rates = np.zeros((len(cache_types), len(cache_sizes)))

    
    for j, cache_size in enumerate(cache_sizes):
        folder_name = f"s6_b4_5{cache_size}4"
        folder_path = os.path.join(base_folder, folder_name)

        if os.path.exists(folder_path):
            
            for i, cache_type in enumerate(cache_types):
                
                file_name = f"{cache_type}.json"
                file_path = os.path.join(folder_path, file_name)

                if os.path.exists(file_path):
                    metrics = compute_metrics(file_path)
                    hit_rates[i, j] = metrics['hit_rate']
                    miss_rates[i, j] = metrics['miss_rate']
                    expiration_rates[i, j] = metrics['expiration_rate']
                else:
                    print(f"File {file_name} not found in {folder_name}")
        else:
            print(f"Folder {folder_path} not found.")

    
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(f"Total number of queries: {metrics['total_queries']}")
    colors = ['b', 'g', 'r']
    markers = ['o', 's', 'D']

    
    for i, cache_type in enumerate(cache_types):
        axes[0].plot(cache_sizes, hit_rates[i], label=f"{cache_type}", marker=markers[i], color=colors[i])
    axes[0].set_title('Cache Hit Rate vs Cache Type and Size')
    axes[0].set_xlabel('Cache Size')
    axes[0].set_ylabel('Cache Hit Rate', labelpad=0, fontweight='bold')
    axes[0].legend()
    axes[0].grid(True)

    
    for i, cache_type in enumerate(cache_types):
        axes[1].plot(cache_sizes, miss_rates[i], label=f"{cache_type}", marker=markers[i], color=colors[i])
    axes[1].set_title('Cache Miss Rate vs Cache Type and Size')
    axes[1].set_xlabel('Cache Size')
    axes[1].set_ylabel('Cache Miss Rate', labelpad=0, fontweight='bold')
    axes[1].legend()
    axes[1].grid(True)

    
    for i, cache_type in enumerate(cache_types):
        axes[2].plot(cache_sizes, expiration_rates[i], label=f"{cache_type}", marker=markers[i], color=colors[i])
    axes[2].set_title('Expiration Rate vs Cache Type and Size')
    axes[2].set_xlabel('Cache Size')
    axes[2].set_ylabel('Expiration Rate', labelpad=0, fontweight='bold')
    axes[2].legend()
    axes[2].grid(True)

    
    plt.tight_layout()

    
    plt.show()

if __name__ == "__main__":
    main()
