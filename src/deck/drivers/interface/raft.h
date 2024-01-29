#ifndef __RAFT_H__
#define __RAFT_H__
#include <stdint.h>
#include <stdbool.h>
#include "semphr.h"
#include "routing.h"

#define RAFT_DEBUG_ENABLE

/* Raft Constants */
#define RAFT_LOG_SIZE_MAX 100
#define RAFT_CLUSTER_PEER_NODE_COUNT_MAX 10
#define RAFT_VOTE_FOR_NO_ONE UWB_DEST_EMPTY

typedef enum {
  FOLLOWER, CANDIDATE, LEADER
} RAFT_STATE;

typedef struct {
  uint16_t term;
  uint16_t index;
  char *command; // TODO: COMMAND enum type definition
} Raft_Log_t;

typedef struct {
  SemaphoreHandle_t mu;
  UWB_Address_t peerNodes[RAFT_CLUSTER_PEER_NODE_COUNT_MAX]; /* peer nodes in current raft cluster configuration */
  uint8_t voteCount; /* granted vote count from peer nodes in current term */
  RAFT_STATE currentState; /* latest term server has seen (initialized to 0 on first boot, increases monotonically) */
  uint16_t currentTerm; /* latest term server has seen (initialized to 0 on first boot, increases monotonically) */
  UWB_Address_t voteFor; /* candidate that received vote in current term (or null if none), RAFT_VOTE_FOR_NO_ONE == null */
  Raft_Log_t log[RAFT_LOG_SIZE_MAX]; /* log entries, each entry contains command for state machine, and term when entry was received by leader (first index is 1) */
  uint16_t commitIndex; /* index of highest log entry known to be committed (initialized to 0, increases monotonically) */
  uint16_t lastApplied; /* index of highest log entry known to be applied to state machine (initialized to 0, increases monotonically) */
  uint16_t nextIndex[RAFT_CLUSTER_PEER_NODE_COUNT_MAX]; /* for each server, index of the next log entry to send to that server (initialized to leader last log index + 1) */
  uint16_t matchIndex[RAFT_CLUSTER_PEER_NODE_COUNT_MAX]; /* for each server, index of highest log entry known to be replicated on server (initialized to 0, increases monotonically) */
  Time_t lastHeartbeatTime; /* heartbeat used for trigger leader election */
} Raft_Node_t;

typedef struct {
  uint16_t term; /* candidate's term */
  UWB_Address_t candidateId; /* candidate that requesting vote */
  uint16_t lastLogIndex; /* index of candidate's last log entry */
  uint16_t lastLogTerm; /* term of candidate's last log entry */
} __attribute__((packed)) Raft_Request_Vote_Args_t;

typedef struct {
  uint16_t term; /* currentTerm, for candidate to update itself */
  bool voteGranted; /* true means candidate received vote */
} __attribute__((packed)) Raft_Request_Vote_Reply_t;

#define RAFT_LOG_ENTRIES_SIZE_MAX ((ROUTING_DATA_PACKET_PAYLOAD_SIZE_MAX - 8) / sizeof (Raft_Log_t))

typedef struct {
  uint16_t term; /* leader's term */
  UWB_Address_t leaderId; /* so follower can redirect clients */
  uint16_t prevLogIndex; /* index of log entry immediately preceding new ones */
  uint16_t prevLogTerm; /* term of prevLogIndex entry */
  Raft_Log_t entries[RAFT_LOG_ENTRIES_SIZE_MAX]; /* log entries to store (empty for heartbeat; may send more than one for efficiency) */
  uint16_t leaderCommit; /* leader's commitIndex */
} __attribute__((packed)) Raft_Append_Entries_Args_t;

typedef struct {
  uint16_t term; /* currentTerm, for leader to update itself */
  bool success; /* true if follower contained entry matching prevLogIndex and prevLogTerm */
} __attribute__((packed)) Raft_Append_Entries_Reply_t;

void raftInit(Raft_Node_t *node);
void raftSendRequestVote(UWB_Address_t address, Raft_Request_Vote_Args_t *args);
void raftProcessRequestVoteReply(Raft_Request_Vote_Reply_t *reply);
void raftSendAppendEntries(UWB_Address_t address, Raft_Append_Entries_Args_t *args);
void raftProcessAppendEntriesReply(Raft_Append_Entries_Reply_t *reply);

#endif