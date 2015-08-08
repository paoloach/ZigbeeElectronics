/**************************************************************************************************
  Filename:       ClusterGroups.c

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#include "aps_groups.h"

#include "ClusterGroups.h"


uint8 groupNameSupported=0;

static uint8 seqNum;
static uint8 srcEP;

static ZStatus_t sendGroupResponse(afAddrType_t *dstAddr,
                                        uint8 cmd, uint8 status, uint16 groupID,
                                        uint8 disableDefaultRsp){
	uint8 buf[3];

	buf[0] = status;
	buf[1] = LO_UINT16( groupID );
 	buf[2] = HI_UINT16( groupID );

 	return zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_GROUPS,
                          cmd, TRUE, ZCL_FRAME_SERVER_CLIENT_DIR,
                          disableDefaultRsp, 0, seqNum, 3, buf );
}

static void sendGroupViewResponse( afAddrType_t *dstAddr,
                 uint8 status, aps_Group_t *grp, uint8 disableDefaultRsp ){
	uint8 *buf;
 	uint8 len;

 	len = 1 + 2; // Status + Group ID

 	if ( status == ZCL_STATUS_SUCCESS )
    	len += grp->name[0] + 1;  // String + 1 for length

 	buf = osal_mem_alloc( len );
 	if ( buf ) {
		buf[0] = status;
 		buf[1] = LO_UINT16( grp->ID );
 		buf[2] = HI_UINT16( grp->ID );

 		if ( status == ZCL_STATUS_SUCCESS ){
			buf[3] = grp->name[0]; // string length
      		osal_memcpy( &buf[4], (&grp->name[1]), grp->name[0] );
    	}

   		zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_GROUPS,
                            COMMAND_GROUP_VIEW_RSP, TRUE, ZCL_FRAME_SERVER_CLIENT_DIR,
                            disableDefaultRsp, 0, seqNum, len, buf );
    	osal_mem_free( buf );
  	}
 }

static ZStatus_t sendGroupGetMembershipRequest( afAddrType_t *dstAddr,
                              uint8 cmd, uint8 rspCmd, uint8 direction, uint8 capacity,
                              uint8 grpCnt, uint16 *grpList, uint8 disableDefaultRsp ){
	uint8 *buf;
	uint8 *pBuf;
	uint8 len = 0;
 	uint8 i;
 	ZStatus_t status;

 	if ( rspCmd )
    	len++;  // Capacity

  	len++;  // Group Count
  	len += sizeof ( uint16 ) * grpCnt;  // Group List

  	buf = osal_mem_alloc( len );
  	if ( buf ) {
		pBuf = buf;
    	if ( rspCmd )
      		*pBuf++ = capacity;

    	*pBuf++ = grpCnt;
    	for ( i = 0; i < grpCnt; i++ ) {
			*pBuf++ = LO_UINT16( grpList[i] );
 			*pBuf++ = HI_UINT16( grpList[i] );
 		}

    	status = zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_GROUPS,
                              cmd, TRUE, direction,
                              disableDefaultRsp, 0, seqNum, len, buf );
    	osal_mem_free( buf );
  	} else
    	status = ZMemError;

  	return status;
}


  

ZStatus_t processGroupsClusterServerCommands(zclIncoming_t *pInMsg) {
	aps_Group_t group;
	uint8 grpCnt;
	uint16 *grpList;
	uint8 grpRspCnt = 0;
	ZStatus_t response;
	uint8 *pData;
	aps_Group_t *pGroup;

	afAddrType_t *dstAddr;
	
	pData = pInMsg->pData;
	group.ID = BUILD_UINT16( pData[0], pData[1] );
	dstAddr =  &pInMsg->msg->srcAddr;
	srcEP = pInMsg->msg->endPoint;
	seqNum = pInMsg->hdr.transSeqNum;
	switch(pInMsg->hdr.commandID){
	case COMMAND_GROUP_ADD:
		response = aps_AddGroup( srcEP,& group );
		if (response != ZSuccess){
			if ( response == ZApsDuplicateEntry )
				response = ZCL_STATUS_DUPLICATE_EXISTS;
        	else
	          response=ZCL_STATUS_INSUFFICIENT_SPACE;
		}
		sendGroupResponse( dstAddr, COMMAND_GROUP_ADD_RSP, response, group.ID, TRUE);
		response = ZCL_STATUS_CMD_HAS_RSP;
		break;
	case COMMAND_GROUP_VIEW:
		 pGroup = aps_FindGroup( srcEP, group.ID );
      	if ( pGroup ) {
	        response = ZCL_STATUS_SUCCESS;
      	} else {
        	// Group not found
        	response = ZCL_STATUS_NOT_FOUND;
        	pGroup = &group;
      	}
      	sendGroupViewResponse( dstAddr, response, pGroup, true );
      	response = ZCL_STATUS_CMD_HAS_RSP;
		break;
	case COMMAND_GROUP_GET_MEMBERSHIP:
		grpCnt = *pData++;
        
      	// Allocate space for the group list
      	grpList = osal_mem_alloc( sizeof( uint16 ) * APS_MAX_GROUPS );
      	if ( grpList != NULL ) {
	        if ( grpCnt == 0 ) {
	          	// Find out all the groups of which the endpoint is a member.
          		grpRspCnt = aps_FindAllGroupsForEndpoint( srcEP, grpList );
        	} else {
          		// Find out the groups (in the list) of which the endpoint is a member.
          		for ( int i = 0; i < grpCnt; i++ ){
		            group.ID = BUILD_UINT16( pData[0], pData[1] );
            		pData += 2;

            		if ( aps_FindGroup( srcEP, group.ID ) )
              			grpList[grpRspCnt++] = group.ID;
          		}
        	}
      
        	if ( grpCnt == 0 ||  grpRspCnt != 0 )  {
				sendGroupGetMembershipRequest(
					dstAddr, COMMAND_GROUP_GET_MEMBERSHIP_RSP,
					TRUE, ZCL_FRAME_SERVER_CLIENT_DIR, aps_GroupsRemaingCapacity(),
					grpRspCnt, grpList, true );
			}

			osal_mem_free( grpList );
      	} else {
	        // Couldn't allocate space for the group list -- send a Default Response command back.
    	    zclDefaultRspCmd_t defaultRspCmd;
        
        	defaultRspCmd.commandID = pInMsg->hdr.commandID;
	        defaultRspCmd.statusCode = ZCL_STATUS_INSUFFICIENT_SPACE;
    	    zcl_SendDefaultRspCmd( srcEP, dstAddr, pInMsg->msg->clusterId, &defaultRspCmd, ZCL_FRAME_SERVER_CLIENT_DIR, true, 0, seqNum );
		}

      	response = ZCL_STATUS_CMD_HAS_RSP;
		break;
	case COMMAND_GROUP_REMOVE:
		if ( aps_RemoveGroup( pInMsg->msg->endPoint, group.ID ) )
        	response = ZCL_STATUS_SUCCESS;
      	else
        	response = ZCL_STATUS_NOT_FOUND;
		sendGroupResponse( dstAddr, COMMAND_GROUP_REMOVE_RSP, response,  group.ID, true );
      	response = ZCL_STATUS_CMD_HAS_RSP;
		break;
	case COMMAND_GROUP_REMOVE_ALL:
	case COMMAND_GROUP_ADD_IF_IDENTIFYING:
	default:
		response = ZFailure;
	}
	return response;
}