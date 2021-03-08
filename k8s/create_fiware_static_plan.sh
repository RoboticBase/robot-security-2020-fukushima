#!/bin/bash

export FIWARE_SERVICE="uoapoc2020"
export FIWARE_SERVICEPATH="/"
export type=plan

# point
p1=(37.6307075583 141.014775192)
p2=(37.6307685533 141.014775632)
p3=(37.6307695 141.0146828)
p4=(37.6307675583 141.014807592)
p5=(37.630870115 141.014811423)
p6=(37.6312103967 141.014817323)
p7=(37.63121135 141.014781435)
p8=(37.6312078833 141.014851263)
p9=(37.6311116467 141.014848578)
p10=(37.630765695 141.01484295)

# function
iso8601() {
  echo $(date '+%Y-%m-%dT%H:%M:%S%z')
}

# delete entities
EXISTING_PLANS=$(curl -sS -H "Fiware-Service: ${FIWARE_SERVICE}" -H "Fiware-Servicepath: ${FIWARE_SERVICEPATH}" "http://orion:1026/v2/entities/?type=${type}&limit=1000&attrs=id")
LEN=$(echo ${EXISTING_PLANS} | jq length)
count=0
if [ ${LEN} -gt 0 ]; then
  for i in $(seq 0 $((${LEN} - 1))); do
    id=$(echo ${EXISTING_PLANS} | jq .[${i}].id -r)
    echo "${id} will be deleted."
    curl -i -H "Fiware-Service: ${FIWARE_SERVICE}" -H "Fiware-Servicepath: ${FIWARE_SERVICEPATH}" "http://orion:1026/v2/entities/${id}/?type=${type}" -X DELETE
    let ++count
  done
fi
echo "${count} entities were deleted."

# register entity
PLANS=$(cat << __EOD__
[
  {
    "plan_id": "plan01",
    "waypoints": [
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p3[0]},
                        "longitude": ${p3[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 15
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 5
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": 90
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 5,
                        "map": "GPS"
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p4[0]},
                        "longitude": ${p4[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p5[0]},
                        "longitude": ${p5[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 2
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p6[0]},
                        "longitude": ${p6[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p7[0]},
                        "longitude": ${p7[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 0
                    }
                }
    ]
  },
  {
    "plan_id": "plan01r",
    "waypoints": [
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p6[0]},
                        "longitude": ${p6[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p8[0]},
                        "longitude": ${p8[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p9[0]},
                        "longitude": ${p9[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 2
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p10[0]},
                        "longitude": ${p10[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p4[0]},
                        "longitude": ${p4[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 5
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": 270
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 5,
                        "map": "Cartographer"
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p3[0]},
                        "longitude": ${p3[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                        "delay": 15
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p2[0]},
                        "longitude": ${p2[1]}
                    },
                    "angle": {
                        "theta": null
                    },
                    "speed": null,
                    "metadata": {
                    }
                },
                {
                    "point": {
                        "altitude": 0,
                        "latitude": ${p1[0]},
                        "longitude": ${p1[1]}
                    },
                    "angle": {
                        "theta": 0
                    },
                    "speed": null,
                    "metadata": {
                    }
                }
    ]
  }
]
__EOD__
)

LEN=$(echo ${PLANS} | jq length)
count=0
if [ ${LEN} -gt 0 ]; then
  for i in $(seq 0 $((${LEN} - 1))); do
    plan_id=$(echo ${PLANS} | jq -r .[${i}].plan_id)
    waypoints=$(echo ${PLANS} | jq -r .[${i}].waypoints)
    echo "${plan_id} will be registerd."
curl -i "http://orion:1026/v2/entities" \
     -H "Fiware-Service: ${FIWARE_SERVICE}" \
     -H "Fiware-ServicePath: ${FIWARE_SERVICEPATH}" \
     -H "Content-Type: application/json" \
     -X POST -d @- <<__EOS__
{
  "id": "${plan_id}",
  "type": "${type}",
  "TimeInstant": {
    "type": "ISO8601",
    "value": "$(iso8601)"
  },
  "waypoints": {
    "type": "array",
    "value": ${waypoints}
  }
}
__EOS__
    let ++count
  done
fi
echo "${count} entities were registered."

