import paramiko
import logging

ssh_host = 'youbot'
ssh_user = 'root'  # <- replace with your SSH username

try:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.load_system_host_keys()

    # private_key = paramiko.RSAKey.from_private_key_file('/home/pi/.ssh/id_rsa_new')  # Update this path
    ssh.connect(ssh_host, username=ssh_user)
    command = (
        f"in_docker '{""}' '{""} |& tee {""}'"
    )
    stdin, stdout, stderr = ssh.exec_command('hostname')
    print("STDOUT:", stdout.read().decode())
    print("STDERR:", stderr.read().decode())    
    ssh.close()

except Exception as e:
    print(f"SSH connect failed: {e}")
