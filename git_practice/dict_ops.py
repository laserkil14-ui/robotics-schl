# dict_ops.py
def add(foo, bar):
    out = {}
    for key in foo.keys():
        if key in bar:
            out[key] = foo[key] + bar[key]
    return out

def subtract(foo, bar):
    out = {}
    for key in foo.keys():
        if key in bar:
            out[key] = foo[key] - bar[key]
    return out

def multiply(foo, bar):
    out = {}
    for key in foo.keys():
        if key in bar:
            out[key] = foo[key] * bar[key]
    return out

def divide(foo, bar):
    out = {}
    for key in foo.keys():
        if key in bar:
            out[key] = foo[key] / bar[key]
    return out
    # main.py
import list_ops as lo
import dict_ops as do

if __name__ == "__main__":
    foo = [1,2,3]
    bar = [4,5,6]
    print(f"foo: {foo}, bar: {bar}")
    print("foo+bar=", lo.add(foo, bar))
    print("foo-bar=", lo.subtract(foo, bar))
    print("foo*bar=", lo.multiply(foo, bar))
    print("foo/bar=", lo.divide(foo, bar))
    
    foo = {"Java": 79, "Cpp": 45, "Python": 99}
    bar = {"Java": 36, "Python": 56, "Ruby": 63}
    print(f"foo: {foo}, bar: {bar}")
    print("foo+bar=", do.add(foo, bar))
    print("foo-bar=", do.subtract(foo, bar))
    print("foo*bar=", do.multiply(foo, bar))
    print("foo/bar=", do.divide(foo, bar))
