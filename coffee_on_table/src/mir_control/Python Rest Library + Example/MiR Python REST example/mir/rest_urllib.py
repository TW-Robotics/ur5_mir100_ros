# MiR - REST urllib v1.0.0
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


import urllib2
import json


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


    def get(self, url, params=None, timeout=None): #TODO: make use of params!
        if not timeout:
            timeout = self._timeout
        url = self._construct_url(url)
        if params is not None:
            url=url+'?'+params
        headers = self._construct_headers()

        try:
            req = urllib2.Request(url=url, headers=headers)
            handler = urllib2.urlopen(req, timeout=timeout)
            status_code = handler.getcode()
            response = json.loads(handler.read())
        except urllib2.HTTPError as e:
            print 'The server couldn\'t fulfill the request. Most likely wrong input.'
            print 'Error code: ', e.code
            status_code = e.code
            response = ''
        except urllib2.URLError as e:
            print 'Failed to reach the server. Most likely wrong ip.'
            print 'Reason: ', e.reason
            response = e.reason
            status_code = 0
        except:
            status_code = 0
            response = ''

        return status_code, response


    def put(self, url, params=None, data=None, timeout=None, allow_insert=True): #allow_insert is not used
        if not timeout:
            timeout = self._timeout
        if data is not None:
            data = json.dumps(data)
        url = self._construct_url(url)
        headers = self._construct_headers()
        headers['Content-Type'] = 'application/json'

        try:
            req = urllib2.Request(url=url, data=data, headers=headers)
            req.get_method = lambda: 'PUT'
            handler = urllib2.urlopen(req, timeout=timeout)
            status_code = handler.getcode()
            response = json.loads(handler.read())
        except urllib2.HTTPError as e:
            print 'The server couldn\'t fulfill the request. Most likely wrong input.'
            print 'Error code: ', e.code
            status_code = e.code
            response = ''
        except urllib2.URLError as e:
            print 'Failed to reach the server. Most likely wrong ip.'
            print 'Reason: ', e.reason
            response = e.reason
            status_code = 0
        except:
            status_code = 0
            response = ''

        return status_code, response


    def delete(self, url, params=None, timeout=None):
        if not timeout:
            timeout = self._timeout
        url = self._construct_url(url)
        headers = self._construct_headers()

        try:
            req = urllib2.Request(url=self._construct_url(url), headers=headers)
            req.get_method = lambda: 'DELETE'
            handler = urllib2.urlopen(req, timeout=timeout)
            status_code = handler.getcode()
            response = json.loads(handler.read())
        except urllib2.HTTPError as e:
            print 'The server couldn\'t fulfill the request. Most likely wrong input.'
            print 'Error code: ', e.code
            status_code = e.code
            response = ''
        except urllib2.URLError as e:
            print 'Failed to reach the server. Most likely wrong ip.'
            print 'Reason: ', e.reason
            response = e.reason
            status_code = 0
        except:
            status_code = 0
            response = ''

        return status_code, response


    def post(self, url, params=None, data=None, timeout=None, allow_update=True): #allow_update is not used
        if not timeout:
            timeout = self._timeout
        if data is not None:
            data = json.dumps(data)
        url = self._construct_url(url)
        headers = self._construct_headers()
        headers['Content-Type'] = 'application/json'

        try:
            req = urllib2.Request(url=url, headers=headers, data=data)
            handler = urllib2.urlopen(req, timeout=timeout)
            status_code = handler.getcode()
            response = json.loads(handler.read())
        except urllib2.HTTPError as e:
            print 'The server couldn\'t fulfill the request. Most likely wrong input.'
            print 'Error code: ', e.code
            status_code = e.code
            response = ''
        except urllib2.URLError as e:
            print 'Failed to reach the server. Most likely wrong ip.'
            print 'Reason: ', e.reason
            response = e.reason
            status_code = 0
        except:
            status_code = 0
            response = ''

        return status_code, response
