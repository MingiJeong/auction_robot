#!/usr/bin/env python

def none_in_dict(d):
    """
    function to check whether there is "None" in dict value
    """
    for key, value in d.items():
        if value is None:
            return True
    return False


# def 