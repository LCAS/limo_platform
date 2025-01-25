# A containerised, clean ROS2 workspace for the AgileX Limo Platform

## Using the Docker image for the real robot

### Starting the Container

1. Navigate to the `configs` directory of this repository:
```bash
cd configs
```

2. Start the container in detached mode and view logs:
```bash
docker compose up -d && docker compose logs -f
```

*Note:* By default, the containers here are restarted at boot time

### Accessing the virtual desktop

1. Find the IP address of your robot, let's name it `LIMO_IP`
2. Open your browser at address http://`LIMO_IP`:5801/, and connect


### Stopping the container

1. `docker compose down`, again in the `configs` directory