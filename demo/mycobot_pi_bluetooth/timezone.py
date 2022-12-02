import sys, subprocess
sys.path.append('.')


def getLocaleTimeZone() -> str:
    resp: str = subprocess.run(['ls -l /etc/localtime'],
                                     stdout=subprocess.PIPE,
                                     shell=True).stdout.decode('utf-8')
    t = resp.split('->')[1]
    l = t.split('/')
    timezone = f'{l[-2]}/{l[-1]}'.replace('\n','').replace('\r','')
    return timezone

def setTimeZone(timezone:str):
    subprocess.run([f'sudo timedatectl set-timezone {timezone}'],
                                     stdout=subprocess.PIPE,
                                     shell=True)

if __name__ == "__main__":
    getLocaleTimeZone()
