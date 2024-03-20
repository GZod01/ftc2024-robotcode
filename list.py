

def calcul_suite(n):
    res1 = [1];
    for i in range(n):
        res2 = 0;
        for j in res1:
            res2+=j;
        res2+=1;
        res1.append(res2);
    return res1[n-1];

