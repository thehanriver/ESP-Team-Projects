import subprocess
import os

def myrun(cmd):
    """from http://blog.kagesenshi.org/2008/02/teeing-python-subprocesspopen-output.html
    """
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines = True)
    stdout = []
    while True:
        line = p.stdout.readline()
        print(line)
        with open('../data/sensors.csv','a') as file:
            file.write(line)
        stdout.append(line)

        if line == '' and p.poll() != None:
            break
    return ''.join(stdout)

if __name__ == "__main__":
    if not os.path.exists('../data'):
        os.makedirs('../data')
    os.system("rm ../data/sensors.csv")
    myrun("node demo.js")
