def multiply(a,b):
    print("Will compute", a, "times", b)
    c = 0
    for i in range(0, int(a)):
        c = c + int(b)
    return c