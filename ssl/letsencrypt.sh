#!/bin/bash

# CONFIGURATION
DUCKDNS_DOMAIN="mysdr"  # just the subdomain, not the full domain
DUCKDNS_TOKEN="your-duckdns-token-here"
CERT_DIR="/etc/letsencrypt/live/${DUCKDNS_DOMAIN}.duckdns.org"
EMAIL="you@example.com"  # for Let's Encrypt notifications

# Install Certbot + DuckDNS plugin
sudo apt update
sudo apt install -y certbot

# Create the DuckDNS credentials file for Certbot
mkdir -p ~/.duckdns
cat > ~/.duckdns/duckdns.ini <<EOF
dns_duckdns_token = ${DUCKDNS_TOKEN}
EOF

chmod 600 ~/.duckdns/duckdns.ini

# Get the wildcard certificate
sudo certbot certonly \
  --manual \
  --preferred-challenges dns \
  --manual-auth-hook "curl -s \"https://www.duckdns.org/update?domains=${DUCKDNS_DOMAIN}&token=${DUCKDNS_TOKEN}&txt=\$CERTBOT_VALIDATION\"" \
  --manual-cleanup-hook "curl -s \"https://www.duckdns.org/update?domains=${DUCKDNS_DOMAIN}&token=${DUCKDNS_TOKEN}&txt=\"" \
  --email "$EMAIL" --agree-tos --no-eff-email \
  -d "${DUCKDNS_DOMAIN}.duckdns.org" -d "*.duckdns.org"

# Reminder
echo "? If successful, your certs will be in: $CERT_DIR"
echo "Use cert.pem and privkey.pem for /home/pi/sbitx/ssl"
