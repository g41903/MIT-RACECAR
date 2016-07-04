from collections import MutableMapping

class keyeddict(MutableMapping):
    """
    A dictionary with a custom key function, ie:

    d = keyeddict(key_func=id)

    will create a dictionary where all lookups are done by looking at the id
    """
    def __init__(self, *args, **kwargs):
        self._key_func = kwargs.pop('key_func')
        self._dict = dict(*args, **kwargs)

    def __getitem__(self, key):
        return self._dict[self._key_func(key)]

    def __setitem__(self, key, value):
        self._dict[self._key_func(key)] = value

    def __delitem__(self, key):
        del self._dict[self._key_func(key)]

    def __contains__(self, key):
        return self._key_func(key) in self._dict

    def __iter__(self):
        return (self._key_func(key) for key in self._dict)

    def __len__(self):
        return len(self._dict)

    def values(self):
        return self._dict.values()


