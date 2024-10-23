import matplotlib.pyplot as plt
import numpy as np

def plot_average_rate_per_balloon(data: dict, balloon_responses: dict):
    plt.figure(figsize=(15, 8))

    hit_rates = []
    miss_rates = []
    expired_rates = []
    balloons = []

    for balloon, responses in balloon_responses.items():
        total = len(responses)
        hit_rate = responses.count(1) / total * 100
        miss_rate = responses.count(0) / total * 100
        expired_rate = responses.count(-1) / total * 100
        
        hit_rates.append(hit_rate)
        miss_rates.append(miss_rate)
        expired_rates.append(expired_rate)
        balloons.append(balloon)

    
    bar_width = 0.25
    indices = np.arange(len(balloons))

    
    plt.bar(indices, hit_rates, bar_width, label='Hit Rate (%)', color='green')
    plt.bar(indices + bar_width, miss_rates, bar_width, label='Miss Rate (%)', color='blue')
    plt.bar(indices + 2 * bar_width, expired_rates, bar_width, label='Expired Rate (%)', color='red')

    
    plt.xlabel('Balloons', fontsize=14)
    plt.ylabel('Percentage (%)', fontsize=14)
    plt.title('Average Cache Hit/Miss/Expired Rate per Balloon', fontsize=16, color="darkgreen")
    
    
    metadata_text = f"Sensors: {data['sensors']} | Cache Type: {data['cache_type']} | Cache Expiration: {data['cache_expiration']} | Cache Size: {data['cache_size']}"
    plt.suptitle(metadata_text, fontsize=12)

    
    plt.xticks(indices + bar_width, balloons)

    plt.legend()
    plt.grid(True)

    plt.savefig(f"/home/intou/Desktop/IoT/IoT-Project-2024/images/rate_per_balloon/{data['sensors']}_{data['cache_type']}_{data['cache_expiration']}_{data['cache_size']}.pdf", format='pdf')
    plt.tight_layout()
    plt.show()
