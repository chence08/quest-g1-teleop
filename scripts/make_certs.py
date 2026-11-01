#!/usr/bin/env python3
"""Create a project-local CA and server certificate; does not change system trust."""
import argparse
import ipaddress
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(*args):
    subprocess.run(['openssl', *args], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', action='append', type=ipaddress.ip_address, required=True,
                        help='LAN IP to include in certificate; repeat for PC2 or additional NICs')
    parser.add_argument('--replace-server', action='store_true', help='replace the server certificate, retaining the existing CA')
    args = parser.parse_args()
    certs = ROOT / 'certs'
    certs.mkdir(exist_ok=True)
    os.umask(0o077)
    ca_key, ca_cert = certs / 'rootCA.key', certs / 'rootCA.pem'
    key, cert = certs / 'key.pem', certs / 'cert.pem'
    if (key.exists() or cert.exists()) and not args.replace_server:
        parser.error('server files already exist; use --replace-server to regenerate them for a new IP')
    if ca_key.exists() != ca_cert.exists():
        parser.error('incomplete CA files; inspect certs/ before proceeding')
    if not ca_key.exists():
        run('req', '-x509', '-newkey', 'rsa:3072', '-nodes', '-days', '365',
            '-keyout', str(ca_key), '-out', str(ca_cert), '-sha256',
            '-subj', '/CN=Quest G1 Local Teleop CA',
            '-addext', 'basicConstraints=critical,CA:TRUE',
            '-addext', 'keyUsage=critical,keyCertSign,cRLSign')
    with tempfile.TemporaryDirectory(dir=certs) as temporary:
        temp = Path(temporary)
        san = ['DNS:localhost', 'IP:127.0.0.1'] + [f'IP:{address}' for address in args.ip]
        extensions = temp / 'server.ext'
        extensions.write_text('basicConstraints=critical,CA:FALSE\n'
                              'keyUsage=critical,digitalSignature,keyEncipherment\n'
                              'extendedKeyUsage=serverAuth\nsubjectAltName=' + ','.join(san) + '\n')
        run('req', '-new', '-newkey', 'rsa:3072', '-nodes', '-keyout', str(key),
            '-out', str(temp / 'server.csr'), '-subj', '/CN=Quest G1 Teleop')
        run('x509', '-req', '-in', str(temp / 'server.csr'), '-CA', str(ca_cert),
            '-CAkey', str(ca_key), '-CAcreateserial', '-out', str(cert),
            '-days', '365', '-sha256', '-extfile', str(extensions))
    run('verify', '-CAfile', str(ca_cert), str(cert))
    print('Created:', cert)
    print('Trust rootCA.pem on the Quest if using the CA route. Keep rootCA.key private on this workstation.')


if __name__ == '__main__':
    main()
