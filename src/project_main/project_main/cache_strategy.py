from enum import Enum, auto
from cachetools import Cache, FIFOCache, RRCache, LRUCache, LFUCache
import time


class CacheType(Enum):
    FIFO = auto()
    RR = auto()
    LRU = auto()
    LFU = auto()

class CacheExpiredError(KeyError):
    """Exception raised when a cache entry has expired."""
    pass

class KeyNotFoundError(KeyError):
    """Exception raised when a key is not found in the cache."""
    pass

class ExpiringCache:
    """A cache that expires items after a set time and limits the total entries."""
    def __init__(self, cache: Cache, expiration_time: float):
        self.cache = cache
        self.expiration_time = expiration_time  

    def __setitem__(self, key, value):
        """Add a new item to the cache."""
        self.cache[key] = value

    def __getitem__(self, key):
        """Retrieve an item, checking if it has expired."""
        if key in self.cache:
            if time.time() - self.cache[key].misuration_ts < self.expiration_time:
                return self.cache[key]
            else:
                self.cache.pop(key)
                raise CacheExpiredError()
        else:
            raise KeyNotFoundError()

    def clear(self):
        """Clear the cache and timestamp tracking."""
        self.cache.clear()

    def __len__(self):
        return len(self.cache)

    def __repr__(self):
        """String representation of the cache contents and expiration time."""
        cache_contents = {key: value for key, value in self.cache.items()}
        return f"<ExpiringCache(cache={cache_contents}, expiration_time={self.expiration_time}s)>"

class CacheFactory:
    @staticmethod
    def create(typ: str, size: int, expiration_time: float) -> ExpiringCache:
        """Create a cache with a specific type and expiration time."""
        try:
            cache_type = CacheType[typ.upper()]
        except KeyError:
            raise ValueError(f'Invalid cache type: {typ}')

        if cache_type == CacheType.FIFO:
            cache = FIFOCache(maxsize=size)
        elif cache_type == CacheType.RR:
            cache = RRCache(maxsize=size)
        elif cache_type == CacheType.LRU:
            cache = LRUCache(maxsize=size)
        else:
            cache = LFUCache(maxsize=size)

        # Wrap the chosen cache with an expiration layer
        return ExpiringCache(cache, expiration_time)
