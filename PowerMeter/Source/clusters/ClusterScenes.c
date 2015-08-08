/**************************************************************************************************
  Filename:       ClusterScenes.c

  Autorh:  Paolo Achdjia
  Created: 28/10/2014

**************************************************************************************************/

#include "OSAL_nv.h"

#include "ClusterScenes.h"
#include "ClusterLevel.h"
#include "ClusterOnOff.h"

uint8 sceneCount;
uint8 currentScene;
uint16 currentGroup;
uint8 sceneValid;
uint8 nameSupported=0;

static uint8 seqNum;
static uint8 srcEP;
static afAddrType_t *dstAddr;



// The maximum length of the scene extension field:
//   2 + 1 + 1 for On/Off cluster (onOff attibute)
//   2 + 1 + 1 for Level Control cluster (currentLevel attribute)
#define SCENE_EXT_LEN                            8
  
// The maximum number of entries in the Scene table
#define MAX_SCENES                               16

/*********************************************************************
 * TYPEDEFS
 */

// The format of a Scene Table Entry
struct Scene_t {
  uint8  used;						// 1 if used
  uint8 endPoint;
  uint16 groupID;                   // The group ID for which this scene applies
  uint8 ID;                         // Scene ID
  uint16 transTime;                 // Time to take to transition to this scene
  uint8 extLen;                     // Length of extension fields
  uint8 extField[SCENE_EXT_LEN]; // Extension fields
};
typedef struct Scene_t Scene;


Scene sceneTable[MAX_SCENES];


/*********************************************************************
 * @fn          scenesSetDefaultNV
 *
 * @brief       Write the defaults to NV
 *
 * @param       none
 *
 * @return      none
 */
static void scenesSetDefaultNV( void ){
	Scene *sceneIter = sceneTable;
	Scene *sceneEnd = sceneTable + MAX_SCENES;

	for (;sceneIter != sceneEnd; sceneIter++){
		sceneIter->used=0;
	}
	osal_nv_write( ZCD_NV_SCENE_TABLE, 0, sizeof( Scene )*MAX_SCENES, &sceneTable );
}

/*********************************************************************
 * @fn      zclGeneral_ScenesInitNV
 *
 * @brief   Initialize the NV Scene Table Items
 *
 * @param   none
 *
 * @return  number of scenes
 */
uint8 scenesInitNV( void ){
	uint8  status;
	uint16 size;

 	size = (uint16)sizeof ( Scene ) * MAX_SCENES;

	status = osal_nv_item_init( ZCD_NV_SCENE_TABLE, size, NULL );

	if ( status != ZSUCCESS ){
		scenesSetDefaultNV();
	}

	return status;
}
					
		
	
/*********************************************************************
 * @fn          zclGeneral_ScenesRestoreFromNV
 *
 * @brief       Restore the Scene table from NV
 *
 * @param       none
 *
 * @return      none
 */
void scenesRestoreFromNV( void ){
	osal_nv_read( ZCD_NV_SCENE_TABLE, 0, sizeof( Scene )*MAX_SCENES, &sceneTable );
}					
					

/*********************************************************************
 * @fn      storeScene
 *
 * @brief   store the important modifing parameters for this endpoint
 * 			Ih this case, the on/off attribute, and the level attribute
 *          In case of others parameters, it's need modify the code and allocate more room
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static void storeScene(Scene *scene){
	uint8 * dataIter;
	scene->extLen=8;
	
	dataIter = scene->extField;	
	*dataIter++ = LO_UINT16(ZCL_CLUSTER_ID_GEN_ON_OFF);
	*dataIter++ = HI_UINT16(ZCL_CLUSTER_ID_GEN_ON_OFF);
	*dataIter++=1;
	*dataIter++=onOffValue;
	*dataIter++ = LO_UINT16(ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL);
	*dataIter++ = HI_UINT16(ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL);
	*dataIter++=1;
	*dataIter++=currentLevel;
}

/*********************************************************************
 * @fn      sceneRacall
 *
 * @brief   restore the saved parameter for thes endpoint
 *			in this case the on/off attribute and the level attribute
 *
 * @param   pInMsg - pointer to the incoming message
 *
 * @return  ZStatus_t
 */
static void sceneRecall(Scene *scene){
	uint8 * dataIter = scene->extField;
	uint8 * dataEnd = dataIter + scene->extLen;
	uint16 clusterID;
	
	while (dataIter < dataEnd){
		clusterID = BUILD_UINT16(dataIter[0], dataIter[1]);
		if (clusterID == ZCL_CLUSTER_ID_GEN_ON_OFF){
			setLightStatus(dataIter[3]);
			dataIter += 4;
		}
		if (clusterID == ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL){
			setCurrentLevel(dataIter[3]);
			dataIter += 4;
		}
	}
}

/*********************************************************************
 * @fn          scenesWriteNV
 *
 * @brief       Save the Scene Table in NV
 *
 * @param       none
 *
 * @return      none
 */
static void scenesWriteNV(uint8 index){
	osal_nv_write(ZCD_NV_SCENE_TABLE, index*sizeof(Scene), sizeof(Scene), sceneTable+index);
}


static ZStatus_t addScene(Scene *scene ){
	Scene *sceneIter;
	uint8 index;	
	
	sceneIter = sceneTable;

	for(index=0; index < MAX_SCENES; index++, sceneIter++){
		if (!sceneIter->used){
			osal_memcpy( (uint8*)sceneIter, (uint8*)scene, sizeof ( Scene ));
			sceneIter->used=1;
			scenesWriteNV(index);
		  	return ZSuccess;
		}
	}
	return ZMemError;
}


static uint8 removeAllScenes(uint16 groupID ){
	Scene *sceneIter;
	Scene *sceneEnd = sceneTable + MAX_SCENES;
	uint8 removed=0;
		
	for(sceneIter = sceneTable; sceneIter != sceneEnd; sceneIter++){
		if (sceneIter->used && sceneIter->endPoint ==  srcEP && sceneIter->groupID == groupID){
			sceneIter->used=0;
			removed=1;
		}
	}

	return removed;
}

static uint8 removeScene(uint16 groupID, uint8 sceneID ){
	Scene *sceneIter;
	Scene *sceneEnd = sceneTable + MAX_SCENES;
		
	for(sceneIter = sceneTable; sceneIter->used && sceneIter != sceneEnd && (sceneIter->endPoint != srcEP || sceneIter->groupID != groupID || sceneIter->ID != sceneID); sceneIter++);

	if (sceneIter == sceneEnd)
		return 0;
	else {
		sceneIter->used=0;
		return 1;
	}
}

static int8 findScene(uint16 groupID, uint8 sceneID ){
	Scene *sceneIter;
	uint8 index;
	
	sceneIter = sceneTable;
	for(index=0; index < MAX_SCENES; index++, sceneIter++){
		if (sceneIter->used && sceneIter->endPoint == srcEP && sceneIter->groupID == groupID && sceneIter->ID != sceneID){
			return index;
		}
	}
	return 0xFF;
}

static uint8 findAllScenesForGroup(uint16 groupID, uint8 * sceneList) {
	Scene *sceneIter;
	Scene *sceneEnd = sceneTable + MAX_SCENES;
	uint8 * sceneIDIter = sceneList;
	uint8 count=0;

	for(sceneIter = sceneTable;  sceneIter != sceneEnd; sceneIter++){
		if (sceneIter->used && sceneIter->endPoint == srcEP && sceneIter->groupID==groupID){
			*sceneIDIter++=sceneIter->ID;
			count++;
		}
	}
	return count;
}

static uint8 countAllScenes( void ){
	Scene *sceneIter;
	Scene *sceneEnd = sceneTable + MAX_SCENES;
	uint8 cnt =0;
	
	for(sceneIter = sceneTable; sceneIter != sceneEnd; sceneIter++){
		if(sceneIter->used){
			cnt++;
		}
	}
	
	return cnt;
}

static void sendSceneViewResponse(uint8 status, Scene *scene, uint8 disableDefaultRsp ){
	uint8 *buf;
	uint8 *pBuf;
 	uint8 len = 1 + 2 + 1; // Status + Group ID + Scene ID

 	if ( status == ZCL_STATUS_SUCCESS ) {
		len += 2; // Transition Time
    	len += 1; // string + 1 for length name

    	// Add something for the extension field length
    	len += scene->extLen;
 	}

	buf = osal_mem_alloc( len );
	if ( buf ) {
		pBuf = buf;
    	*pBuf++ = status;
    	*pBuf++ = LO_UINT16( scene->groupID );
    	*pBuf++ = HI_UINT16( scene->groupID );
    	*pBuf++ = scene->ID;
    	if ( status == ZCL_STATUS_SUCCESS ) {
			*pBuf++ = LO_UINT16( scene->transTime );
      		*pBuf++ = HI_UINT16( scene->transTime );
      		*pBuf++ = 0;
			if ( scene->extLen > 0 )
		        osal_memcpy( pBuf, scene->extField, scene->extLen );
    	}	

    	zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_SCENES,
                            COMMAND_SCENE_VIEW_RSP, TRUE, ZCL_FRAME_SERVER_CLIENT_DIR,
                            disableDefaultRsp, 0, seqNum, len, buf );
    	osal_mem_free( buf );
  	} 
}


static void sendSceneResponse(uint8 cmd, uint8 status, uint16 groupID, uint8 sceneID, uint8 disableDefaultRsp ){
	uint8 buf[4];
	uint8 len = 1 + 2; // Status + Group ID

	buf[0] = status;
	buf[1] = LO_UINT16( groupID );
 	buf[2] = HI_UINT16( groupID );

 	if ( cmd != COMMAND_SCENE_REMOVE_ALL_RSP ) {
		buf[3] = sceneID;
    	len++;
	}

 	zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_SCENES,
                          cmd, TRUE, ZCL_FRAME_SERVER_CLIENT_DIR,
                          disableDefaultRsp, 0, seqNum, len, buf );
}

/*********************************************************************
 * @fn      zclGeneral_SendSceneGetMembershipResponse
 *
 * @brief   Call to send Scene Get Membership Response Command
 *
 * @param   status - scene command status
 * @param   capacity - remaining capacity of the scene table
 * @param   sceneCnt - number of scenes in the scene list
 * @param   sceneList - list of scene IDs
 * @param   groupID - group ID that scene belongs to
 *
 * @return  ZStatus_t
 */
static void sendSceneGetMembershipResponse(uint8 status, uint8 sceneCnt, uint8 *sceneList,  uint16 groupID){
  	uint8 *buf;
  	uint8 *pBuf;
	uint8 len = 1 + 1 + 2; // Status + Capacity + Group ID;
	uint8 i;
	uint8 capacity;
	
	capacity = MAX_SCENES - countAllScenes();

	if ( status == ZCL_STATUS_SUCCESS ) {
		len++;
		len += sceneCnt; // Scene List (Scene ID is a single octet)
  	}

 	buf = osal_mem_alloc( len );
  	if ( buf ){
		pBuf = buf;
		*pBuf++ = status;
		*pBuf++ = capacity;
		*pBuf++ = LO_UINT16( groupID );
		*pBuf++ = HI_UINT16( groupID );
		if ( status == ZCL_STATUS_SUCCESS ) {
			*pBuf++ = sceneCnt;
			for ( i = 0; i < sceneCnt; i++ )
				*pBuf++ = sceneList[i];
    	}

		zcl_SendCommand( srcEP, dstAddr, ZCL_CLUSTER_ID_GEN_SCENES,  COMMAND_SCENE_GET_MEMBERSHIP_RSP, TRUE,  ZCL_FRAME_SERVER_CLIENT_DIR, true, 0, seqNum, len, buf );
    	osal_mem_free( buf );
	} 
}

ZStatus_t processSceneClusterServerCommands(zclIncoming_t *pInMsg) {
	ZStatus_t response;
	Scene scene;
	Scene * pScene;
	uint8 *pData;
	uint8 nameLen;
	uint8 status;
	uint8 *sceneList;
	uint8 sendRsp;
	uint8 sceneCnt;
	uint8 sceneIndex;
	
	srcEP = pInMsg->msg->endPoint;
	seqNum = pInMsg->hdr.transSeqNum;
	dstAddr = &pInMsg->msg->srcAddr,
	
	pData = pInMsg->pData;
	osal_memset( (uint8*)&scene, 0, sizeof(Scene) );

	scene.groupID = BUILD_UINT16( pData[0], pData[1] );
	pData += 2;   // Move past group ID
 	scene.ID = *pData++;
	scene.extLen = 0;
	scene.endPoint = srcEP;
	scene.transTime = 0;
	
	status = ZSuccess;
	response=ZSuccess;	
	
	switch(pInMsg->hdr.commandID){
	case COMMAND_SCENE_ADD:
		 // Parse the rest of the incoming message
      	scene.transTime = BUILD_UINT16( pData[0], pData[1] );
      	pData += 2;
      	nameLen= *pData++; // Name length
      	pData += nameLen; // move pass name

      	scene.extLen = pInMsg->pDataLen - ( (uint16)( pData - pInMsg->pData ) );
      	if ( scene.extLen > 0 ) {
        	if ( scene.extLen > ZCL_GEN_SCENE_EXT_LEN )
				scene.extLen = ZCL_GEN_SCENE_EXT_LEN;
        	osal_memcpy( scene.extField, pData, scene.extLen );
      	}

      	if (scene.groupID == 0x0000 ||  aps_FindGroup( srcEP, scene.groupID ) != NULL ) {
			sceneIndex = findScene(scene.groupID, scene.ID );
			if (sceneIndex== 0xFF){
				scene.extLen = pInMsg->pDataLen - ( (uint16)( pData - pInMsg->pData ) );
		      	if ( scene.extLen > 0 ) {
        			if ( scene.extLen > ZCL_GEN_SCENE_EXT_LEN )
						scene.extLen = ZCL_GEN_SCENE_EXT_LEN;
		        	osal_memcpy( scene.extField, pData, scene.extLen );
      			}
				if (addScene( &scene ) != ZSuccess){
					status = ZCL_STATUS_INSUFFICIENT_SPACE;
				}
			} else {
				pScene = &sceneTable[sceneIndex];
				pScene->extLen = pInMsg->pDataLen - ( (uint16)( pData - pInMsg->pData ) );
		      	if ( pScene->extLen > 0 ) {
        			if ( pScene->extLen > ZCL_GEN_SCENE_EXT_LEN )
						pScene->extLen = ZCL_GEN_SCENE_EXT_LEN;
        			osal_memcpy( pScene->extField, pData, pScene->extLen );
      			}
				scenesWriteNV(sceneIndex);
			}
      	} else
        	status = ZCL_STATUS_INVALID_FIELD; // The Group is not in the Group Table

		sendSceneResponse(COMMAND_SCENE_ADD_RSP, status, scene.groupID, scene.ID, true);
      	response = ZCL_STATUS_CMD_HAS_RSP;
	break;
	case COMMAND_SCENE_VIEW:
		sceneIndex = findScene(scene.groupID, scene.ID );
      	if (sceneIndex != 0xFF ){
			status = ZCL_STATUS_SUCCESS;
			pScene = &sceneTable[sceneIndex];
		} else {
        	if ( scene.groupID != 0x0000 && aps_FindGroup( srcEP, scene.groupID ) == NULL )  {
				status = ZCL_STATUS_INVALID_FIELD; // The Group is not in the Group Table
        	} else
	        	status = ZCL_STATUS_NOT_FOUND;
        	pScene = &scene;
      	}
      	sendSceneViewResponse(status, pScene, true);
      	response = ZCL_STATUS_CMD_HAS_RSP;
	  break;
	case COMMAND_SCENE_REMOVE:
		if (removeScene(scene.groupID, scene.ID ) ) {
			status = ZCL_STATUS_SUCCESS;
      	} else {
	        if ( aps_FindGroup(srcEP, scene.groupID ) == NULL ) {
				status = ZCL_STATUS_INVALID_FIELD;
        	} else
          		status = ZCL_STATUS_NOT_FOUND;
      	}

		if ( UNICAST_MSG( pInMsg->msg ) ) {
			sendSceneResponse( COMMAND_SCENE_REMOVE_RSP, status, scene.groupID, scene.ID, true );
		}
      	response = ZCL_STATUS_CMD_HAS_RSP;
	  	break;
	case COMMAND_SCENE_REMOVE_ALL:
		if ( scene.groupID == 0x0000 ||  aps_FindGroup( srcEP, scene.groupID ) != NULL ) {
        	removeAllScenes(scene.groupID );
        	status = ZCL_STATUS_SUCCESS;
      	} else
        	status = ZCL_STATUS_INVALID_FIELD;

		if ( UNICAST_MSG( pInMsg->msg ) ) {
			sendSceneResponse( COMMAND_SCENE_REMOVE_ALL_RSP, status, scene.groupID, scene.ID, true );
		}
		response = ZCL_STATUS_CMD_HAS_RSP;
      break;
	case COMMAND_SCENE_STORE:
		if ( scene.groupID == 0x0000 ||  aps_FindGroup(srcEP, scene.groupID ) != NULL ) {
			sceneIndex = findScene(scene.groupID, scene.ID );
			if (sceneIndex == 0xFF){
				if (addScene(&scene ) == ZSuccess){
					storeScene(&scene);
				} else {
					status = ZCL_STATUS_INSUFFICIENT_SPACE;
				}
			} else {
				pScene = &sceneTable[sceneIndex];
				storeScene(pScene);
			}
        } else
        	status = ZCL_STATUS_INVALID_FIELD; 

      	if ( UNICAST_MSG( pInMsg->msg ) ) {
			sendSceneResponse( COMMAND_SCENE_STORE_RSP, status, scene.groupID, scene.ID, true );
      	}
      	response = ZCL_STATUS_CMD_HAS_RSP;
		break;
	case COMMAND_SCENE_RECALL:
		sceneIndex = findScene(scene.groupID, scene.ID );
     	if (sceneIndex != 0xFF ) {
			pScene = &sceneTable[sceneIndex];
			sceneRecall(pScene);
		}
	break;
	case COMMAND_SCENE_GET_MEMBERSHIP:
		sendRsp=false;
		sceneCnt=0;
		if ( scene.groupID == 0x0000 ||  aps_FindGroup(srcEP, scene.groupID ) != NULL ){
	    	sceneList = osal_mem_alloc(MAX_SCENES );
	        if (sceneList != NULL ){
	   	    	sceneCnt = findAllScenesForGroup(scene.groupID, sceneList); 
          		status = ZCL_STATUS_SUCCESS;
          		if ( UNICAST_MSG( pInMsg->msg ) || sceneCnt != 0 ) {
            		sendRsp = TRUE;
          		}
        	} else {
        		status = ZCL_STATUS_INSUFFICIENT_SPACE;
          		sendRsp = TRUE;
        	}
      	}  else  {
        	status = ZCL_STATUS_INVALID_FIELD;
        	sendRsp = TRUE;
      	}

      	if ( sendRsp )  {
	        sendSceneGetMembershipResponse( status, sceneCnt, sceneList, scene.groupID);
      	}
      
      	if (sceneList != NULL )
        	osal_mem_free( sceneList );
      
      	response = ZCL_STATUS_CMD_HAS_RSP;
	default:
		response = ZFailure;
	}
	return response;
}