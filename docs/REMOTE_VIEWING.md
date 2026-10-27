# Viewing from other machines

To view a bridge running on a robot or server, start it with `--serve`:

```bash
ros-rerun-bridge --config <config.yaml> --serve
```

It hosts a gRPC server (port 9876) and a web viewer (port 9090), both on all network interfaces, and prints how to connect. Any number of viewers can connect and disconnect at any time; one that joins late gets the recent history, up to 1 GiB.

## Native viewer

```bash
rerun rerun+http://<host>:9876/proxy
```

Use the bridge's Rerun version (0.40.0), e.g. `uvx --from rerun-sdk==0.40.0 rerun rerun+http://<host>:9876/proxy`, or [the viewer installed with the bridge](INSTALLATION.md#the-rerun-viewer). On the bridge's own machine, add `--port auto`, as the bridge already uses port 9876.

## Browser

```
http://<host>:9090/?url=rerun%2Bhttp%3A%2F%2F<host>%3A9876%2Fproxy
```

The page only connects when given the server in its `url` parameter, as above.

The server only accepts web viewers served from `localhost` or rerun.io. A browser reaching the bridge by any other address needs that origin allowed, with glob patterns:

```bash
ros-rerun-bridge --config <config.yaml> --serve --allow-origin 'http://192.168.1.10:*'
```

## Security

Neither connection is authenticated or encrypted: anyone who can reach the ports can read the stream. `--allow-origin` only controls which web pages a browser lets connect; it does not stop a native viewer or any other client.

Off a trusted network, block ports 9876 and 9090 from other hosts with a firewall on the bridge's machine, and connect through an SSH tunnel to `localhost`, which needs no `--allow-origin`:

```bash
ssh -L 9876:localhost:9876 -L 9090:localhost:9090 <host>
```

The tunnel only protects your own connection: without the firewall, other hosts can still reach the ports directly.

## In Docker

Publish ports 9876 and 9090. Other machines then reach the bridge by the Docker host's address.
