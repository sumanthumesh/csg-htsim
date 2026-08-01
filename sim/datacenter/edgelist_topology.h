// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef EDGELIST_TOPOLOGY
#define EDGELIST_TOPOLOGY
#include "main.h"
#include "randomqueue.h"
#include "pipe.h"
#include "config.h"
#include "loggers.h"
#include "network.h"
#include "topology.h"
#include "logfile.h"
#include "eventlist.h"
#include "switch.h"
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

// An arbitrary topology, loaded from an edgelist.
//
// Unlike FatTreeTopology, nothing about the structure is assumed: the file
// lists nodes and the links between them, and paths are found by search.
// Hosts are numbered 0..Hosts-1 and these numbers are the endpoint ids used
// by get_paths()/get_bidir_paths(), so they must line up with ASTRA-sim's
// NPU ids.  Switches are numbered separately in the file (s0, s1, ...) but
// share one internal node index space with the hosts, with switch S stored
// at index _no_of_hosts + S.
//
// File format (blank lines and #-comments ignored):
//
//     Hosts 8
//     Switches 4
//     Links 12
//
//     # applied to any link or switch that omits the corresponding attribute
//     Default_speed_Gbps 400
//     Default_latency_ns 500
//     Default_queue_bytes 500000
//     Default_switch_latency_ns 300
//
//     # optional per-switch forwarding latency, for switches that differ from
//     # the default.  Charged once per switch traversed on a path.
//     switch s3 switch_latency_ns 100
//
//     # link <endpoint> <endpoint> [speed_Gbps V] [latency_ns V] [queue_bytes V]
//     link h0 s0
//     link h1 s0 speed_Gbps 200
//     link s0 s1 latency_ns 1000 queue_bytes 1000000
//
//     # optional: pin the paths between a host pair instead of searching for
//     # them.  Lists the intermediate switches only, in order.  Repeat the
//     # same host pair to give it several paths.
//     route h0 h5 s0 s3 s1
//
// Each "link" line is bidirectional and yields two directed edges, each with
// its own queue and pipe, so the two directions do not share buffering.
//
// Switch latency is folded into the pipe delay of each link leaving a switch,
// rather than being held on the Switch object.  It has to be: these routes are
// source routes, so a packet traverses queues and pipes only and never calls
// Switch::receivePacket, which is where FatTreeSwitch charges its latency.
// (That is also why Switch_latency_ns has no effect in the fat-tree setup.)

#ifndef QT
#define QT
typedef enum {UNDEFINED, RANDOM, ECN, COMPOSITE, PRIORITY,
              CTRL_PRIO, FAIR_PRIO, LOSSLESS, LOSSLESS_INPUT, LOSSLESS_INPUT_ECN,
              COMPOSITE_ECN, COMPOSITE_ECN_LB, SWIFT_SCHEDULER, ECN_PRIO, AEOLUS, AEOLUS_ECN} queue_type;
typedef enum {UPLINK, DOWNLINK} link_direction;
#endif

// Cap on the number of paths returned per host pair.  Only paths tied at the
// minimum hop count are ever returned, so everything in the set is
// interchangeable; the caller draws one at random per flow.
//
// Defaults to 1 - a single deterministic path per pair, which is what IB's
// subnet-manager-computed forwarding does.  Raise it for ECMP-style spreading.
// TODO: a flow is pinned to one path for its lifetime either way.  If we later
// need a message to use several paths at once, that is subflow_count > 1 in
// the frontend, not a change here.
#define DEFAULT_MAX_PATHS 1

class EdgelistTopology: public Topology {
public:
    // One direction of one link.  The queue drains into the pipe, and the
    // pipe into whatever sits at the far end; a Route is built by walking
    // these in order.
    struct DirectedLink {
        uint32_t src;              // internal node index
        uint32_t dst;              // internal node index
        linkspeed_bps speed;
        simtime_picosec latency;   // wire delay, plus src's switch latency if src is a switch
        mem_b queuesize;
        BaseQueue* queue;
        Pipe* pipe;
    };

    static EdgelistTopology* load(const char* filename, QueueLoggerFactory* logger_factory,
                                  EventList& eventlist, mem_b queuesize,
                                  queue_type qt, queue_type sender_qt);
    static EdgelistTopology* load(istream& file, QueueLoggerFactory* logger_factory,
                                  EventList& eventlist, mem_b queuesize,
                                  queue_type qt, queue_type sender_qt);

    EdgelistTopology(QueueLoggerFactory* logger_factory, EventList* ev, mem_b queuesize,
                     queue_type qt, queue_type sender_qt);
    ~EdgelistTopology();

    // Inherited from Topology.  The reverse argument is ignored: the ASTRA-sim
    // TCP frontend builds its own (empty) return route for ACKs, so a reverse
    // Route built here would be discarded.
    virtual vector<const Route*>* get_bidir_paths(uint32_t src, uint32_t dest, bool reverse);
    virtual vector<uint32_t>* get_neighbours(uint32_t src);
    virtual uint32_t no_of_nodes() const {return _no_of_hosts;}
    virtual void add_switch_loggers(Logfile& log, simtime_picosec sample_period);

    uint32_t no_of_switches() const {return _no_of_switches;}
    uint32_t no_of_links() const {return _no_of_links;}

    void set_max_paths(uint32_t max_paths) {_max_paths = max_paths;}

    vector<Switch*> switches;

private:
    // Parsing.  Two passes are not needed - unlike the netlist format, an
    // edgelist has no forward references - but node counts must be known
    // before any link line is read.
    bool parse(istream& file);
    // linecount is threaded through so diagnostics report real file line
    // numbers rather than offsets from the end of the header.
    bool parse_header(istream& file, string& first_body_line, int& linecount);
    bool parse_switch(const vector<string>& tokens, int linecount);
    bool parse_link(const vector<string>& tokens, int linecount);
    bool parse_route(const vector<string>& tokens, int linecount);

    // "h3" / "s7" -> internal node index.  Returns false on a malformed or
    // out-of-range name.
    bool parse_endpoint(const string& token, uint32_t& node) const;
    bool is_host(uint32_t node) const {return node < _no_of_hosts;}

    // Allocates the queues and pipes for every link recorded during parsing.
    // Deferred until parsing finishes so that "switch" lines may appear after
    // the links they affect - switch latency is baked into pipe delay.
    void init_network();

    // Mirrors FatTreeTopology::alloc_queue/alloc_src_queue for the queue types
    // the ASTRA-sim frontend actually passes (RANDOM switch-side, FAIR_PRIO
    // host-side), so links behave the same as in the fat-tree setup.
    BaseQueue* alloc_queue(QueueLogger* logger, linkspeed_bps speed, mem_b queuesize);
    BaseQueue* alloc_src_queue(QueueLogger* logger, linkspeed_bps speed);

    DirectedLink* find_link(uint32_t src, uint32_t dst) const;

    // Up to _max_paths shortest paths from src to dest, each as a sequence of
    // internal node indices starting at src and ending at dest.
    vector<vector<uint32_t>> shortest_paths(uint32_t src, uint32_t dest) const;

    // Turn a node sequence into a Route of alternating queue/pipe hops.
    // Returns NULL if any hop has no corresponding link.
    Route* build_route(const vector<uint32_t>& nodes) const;

    uint32_t _no_of_hosts;
    uint32_t _no_of_switches;
    uint32_t _no_of_nodes;      // _no_of_hosts + _no_of_switches
    uint32_t _no_of_links;      // undirected links declared in the header
    uint32_t _max_paths;

    linkspeed_bps _default_speed;
    simtime_picosec _default_latency;
    mem_b _default_queuesize;
    simtime_picosec _default_switch_latency;

    // Per-switch forwarding latency, indexed by switch number.  Initialised to
    // _default_switch_latency and overridden by "switch" lines.  Must be fully
    // known before any link is built, since it is folded into pipe delays.
    vector<simtime_picosec> _switch_latencies;

    // Both directions of every link, keyed by (src node, dst node).
    map<pair<uint32_t, uint32_t>, DirectedLink*> _links;

    // Neighbours of each node, for path search.
    vector<vector<uint32_t>> _adjacency;

    // Explicitly pinned paths, keyed by (src host, dst host).  When a pair has
    // an entry, these are used instead of searching.
    map<pair<uint32_t, uint32_t>, vector<vector<uint32_t>>> _pinned_paths;

    QueueLoggerFactory* _logger_factory;
    EventList* _eventlist;
    mem_b _queuesize;           // fallback when the file gives no default
    queue_type _qt;             // switch-side queues
    queue_type _sender_qt;      // host-side (injection) queues
};

#endif
