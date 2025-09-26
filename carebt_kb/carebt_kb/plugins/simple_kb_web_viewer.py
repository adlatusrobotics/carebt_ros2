# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
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

import json
import html
import threading

from http.server import BaseHTTPRequestHandler, HTTPServer
from carebt_kb.plugin_base import PluginBase
from functools import partial


class SimpleWebServer(BaseHTTPRequestHandler):

    def __init__(self, kb, *args, **kwargs):
        self.__kb = kb
        super().__init__(*args, **kwargs)

    def iri_to_name(self, iri: str) -> str:
        fragment = iri.split("#")[-1]
        filename = iri.split("/")[-1].split(".")[0]
        return f"{filename}.{fragment}"

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        str_clazzes = self.__kb.get_classes()
        # start HTML
        self.wfile.write(bytes("<html>\
            <head>\
                <title>careBT web view</title>\
            <style>\
                table, th, td { border: 1px solid black; }\
                table {padding: 10px;}\
            </style>\
            </head><body>", "utf-8"))
        # iterate classes
        for str_clazz in sorted(str_clazzes):
            if not self.__kb.has_subclasses(str_clazz):
                self.wfile.write(bytes(f"<h3>{str_clazz}</h3>", "utf-8"))
                str_individuals = self.__kb.get_individuals_of(str_clazz)
                individuals = self.__kb.read_items(str_individuals)
                # iterate individuals
                for individual in individuals:
                    self.wfile.write(bytes(f"<table border: 1px>", "utf-8"))
                    # add 'iri' and 'is_a' as the first two rows in the table
                    self.wfile.write(bytes(f"<tr><td><b>iri</b></td><td>{individual['iri']}</td></tr>", "utf-8"))
                    self.wfile.write(bytes(f"<tr><td><b>ref</b></td><td>{self.iri_to_name(individual['iri'])}</td></tr>", "utf-8"))
                    self.wfile.write(bytes(f"<tr><td><b>is_a</b></td><td>{individual['is_a']}</td></tr>", "utf-8"))
                    # iterate over the properties of str_clazz (of the individual)
                    for p in self.__kb.get_properties_of_class(str_clazz):
                        value = individual.get(p['name'], '')
                        length_str = f"length: {len(value)}" if not p['functional'] and value is not None else ""
                        type_str = f"{p['type']}[]" if not p['functional'] else p['type']
                        col1 = f"<b>{p['name']}</b><br>type: {type_str}<br>{length_str}"
                        self.wfile.write(bytes(f"<tr><td>{col1}</td><td>{value}</td></tr>", "utf-8"))
                        
                    self.wfile.write(bytes(f"</table>", "utf-8"))

        # end HTML
        self.wfile.write(bytes("</body></html>", "utf-8"))

    def log_message(self, format, *args):
        return


class SimpleKbWebViewer(PluginBase):

    def on_init_callback(self, plugin_name: str):

        self._kb_server.declare_parameter(f'{plugin_name}.host', 'localhost')
        self._kb_server.declare_parameter(f'{plugin_name}.port', 8080)

        host = self._kb_server.get_parameter(
                f'{plugin_name}.host').get_parameter_value().string_value
        port = self._kb_server.get_parameter(
                f'{plugin_name}.port').get_parameter_value().integer_value

        handler = partial(SimpleWebServer, self._kb_server)
        self.__webServer = HTTPServer((host, port), handler)
        self._kb_server.get_logger().info("SimpleKbWebViewer - Server started http://%s:%s" %
                                          (host, port))

        threading.Thread(target=self.__worker, daemon=True).start()

    def __worker(self):
        self.__webServer.serve_forever()
