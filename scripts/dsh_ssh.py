import sys
import paramiko

HOST = "10.68.1.43"
USER = "root"
PWD = "orangepi"


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "echo connected"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        HOST,
        username=USER,
        password=PWD,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )
    stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    sys.stdout.write(out)
    if err:
        sys.stdout.write("[stderr]\n" + err)
    sys.stdout.write(f"\n[exit code: {rc}]\n")
    c.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()