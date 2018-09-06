# MiR - REST requests v1.0.0
"""
LEGAL DISCLAIMER

This example file is provided as is without any guarantees or warranty.
In association with the product, Mobile Industrial Robots ApS makes no warranties
of any kind, either express or implied, including but not limited to warranties
of merchantability, fitness for a particular purpose, of title, or of noninfringement
of third party rights. Use of the product by a user is at the user's risk.

NO REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS OR IMPLIED, OF MERCHANTABILITY,
FITNESS FOR A SPECIFIC PURPOSE, THE PRODUCTS TO WHICH THE INFORMATION MENTIONS MAY BE
USED WITHOUT INFRINGING THE INTELLECTUAL PROPERTY RIGHTS OF OTHERS, OR OF ANY OTHER
NATURE ARE MADE WITH RESPECT TO INFORMATION OR THE PRODUCT TO WHICH INFORMATION MENTIONS.
IN NO CASE SHALL THE INFORMATION BE CONSIDERED A PART OF OUR TERMS AND CONDITIONS OF SALE.
"""

"""
INSTRUCTIONS

This library is a non-standard python package.
Please follow the instructions to install it:
	- Windows:
		1. Download this ZIP file: https://github.com/kennethreitz/requests/tarball/master
		2. Extract it.
		3. Open a command line (Windows key, cmd, ENTER).
		4. Type "python".
		5. Drag the "setup.py" file from the unzipped folder.
		6. Press space and write "install".
		7. Hit ENTER.
		
	- Mac & Linux (requires pip):
		1. Open a terminal.
		2. Write "pip requests install".
		3. Hit ENTER.
"""

import requests
import json
import logging


def HandleRequestErrorsDecorator(f):
    def wrapped_f(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except requests.exceptions.Timeout:
            return (408, None)  # 408 == Request Timeout in HTTP
        except requests.exceptions.RequestException as e:
            print("HTTP request error: %s" % e.message)
            return (503, None)  # 503 == Service unavailable

    return wrapped_f

class Client(object):
    def __init__(self, host, port, version):
        self.host = ('http://' + host + ':' + str(port)).strip('/')
        self.version = version.strip('/')
        self._timeout = 1.0

    def _construct_url(self, parts):
        if isinstance(parts, list):
            parts = '/'.join([p.strip('/') for p in parts])
        else:
            parts = parts.strip('/')

        if parts[:len(self.host)] == self.host:
            return parts
        if parts[:len(self.version)] == self.version:
            return '/'.join([self.host, parts])
        else:
            return '/'.join([self.host, self.version, parts])

    def _construct_headers(self):
        """Common headers are defined here."""

        headers = {}

        return headers

    @HandleRequestErrorsDecorator
    def get(self, url, params=None, timeout=None):
        if not timeout:
            timeout = self._timeout
        url = self._construct_url(url)
        headers = self._construct_headers()
        req = requests.get(url, params=params, timeout=timeout, headers=headers)
        status_code = req.status_code
        try:
            response = req.json()
        except Exception as e:
            response = ''
        return status_code, response

    @HandleRequestErrorsDecorator
    def put(self, url, params=None, data=None, timeout=None, allow_insert=True):
        if data is not None:
            data_json = json.dumps(data)
        if not timeout:
            timeout = self._timeout
        headers = self._construct_headers()
        headers['Content-Type'] = 'application/json'
        req = requests.put(self._construct_url(url), data=data_json, params=params, headers=headers, timeout=timeout)
        if req.status_code != 200 and allow_insert:
            logging.warn("Put failed - trying to insert URL: %s" % (url))
            ret, data = self.post(url.rsplit('/', 1)[0], params=params, data=data, timeout=timeout, allow_update=False)
            if ret == 201:
                ret = 200
            return ret, data

        status_code = req.status_code
        try:
            response = req.json()
        except Exception as e:
            response = ''
        return status_code, response

    @HandleRequestErrorsDecorator
    def delete(self, url, params=None):
        url = self._construct_url(url)
        headers = self._construct_headers()
        req = requests.delete(url, params=params, headers=headers, timeout=self._timeout)
        status_code = req.status_code
        if status_code == 204:
            response = ''
        else:
            try:
                response = req.json()
            except Exception as e:
                response = ''
        return status_code, response

    @HandleRequestErrorsDecorator
    def post(self, url, params=None, data=None, timeout=None, allow_update=True):
        if data is not None:
            data_json = json.dumps(data)
        if not timeout:
            timeout = self._timeout

        headers = self._construct_headers()
        headers['Content-Type'] = 'application/json'
        req = requests.post(self._construct_url(url), params=params, data=data_json, headers=headers, timeout=timeout)

        if allow_update and req.status_code == 409 and 'guid' in data:
            logging.warn("Post failed as entry already existed - Trying to post an update instead..")
            ret, data = self.put("%s/%s" % (url, data['guid']), params=params, data=data, timeout=timeout,
                                 allow_insert=False)
            if ret == 200:
                ret = 201
            return ret, data

        status_code = req.status_code
        try:
            response = req.json()
        except Exception as e:
            response = ''
        return status_code, response
