import json
from decimal import Decimal

def from_json(json_data):
    '''Converts json to python object'''
    python_obj = json.loads(json_data, parse_float=Decimal)
    return python_obj

def to_json(object):
    '''Converts python object to json'''
    json_object = json.dumps(object, ensure_ascii=False)
    return json_object

