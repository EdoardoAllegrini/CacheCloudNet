
def response_to_code(response):
    if response == "CacheExpiredError":
        return -1  
    elif response == "KeyNotFoundError":
        return 0  
    else:
        return 1  

def extract_query_data(data, start_idx=5, end_idx=75):
    balloon_responses = {}
    i = 0
    for query in data["queries"][start_idx:end_idx]:
        for balloon in query:
            if "Balloon_" in balloon:
                if balloon not in balloon_responses:
                    balloon_responses[balloon] = []
                balloon_responses[balloon].append(response_to_code(query[balloon]['data']))
        i += 1

    return balloon_responses, i