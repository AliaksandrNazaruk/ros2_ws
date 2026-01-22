# Copyright 2026 Boris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Broker Config HTTP client.

Minimal HTTP client for fetching broker configuration from Config Service.
"""

from typing import Dict

import requests


class BrokerConfigClient:
    """
    HTTP client for fetching broker configuration from Config Service.

    Minimal implementation - only handles HTTP GET request.
    No retries, no circuit breaker, no business logic.
    """

    def __init__(self, base_url: str, api_key: str, timeout: float):
        """
        Initialize client.

        Parameters
        ----------
        base_url
            Base URL of Config Service (e.g., 'https://mqtt.techvisioncloud.pl').
        api_key
            API key for authentication (X-API-Key header).
        timeout
            Request timeout in seconds.

        """
        # Request password explicitly (required for MQTT connection)
        self._url = base_url.rstrip('/') + '/api/v1/config/broker?include_password=true'
        self._headers = {
            'Accept': 'application/json',
            'X-API-Key': api_key,
        }
        self._timeout = timeout

    def fetch(self) -> Dict:
        """
        Fetch broker configuration from Config Service.

        Returns
        -------
        Dict
            Dictionary with broker configuration.

        Raises
        ------
        requests.exceptions.RequestException
            On HTTP or network errors.

        """
        resp = requests.get(self._url, headers=self._headers, timeout=self._timeout)
        resp.raise_for_status()
        return resp.json()
