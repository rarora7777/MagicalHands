using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GestureAnim
{
    public class UIManager : MonoBehaviour
    {
        // Start is called before the first frame update

        public GameObject UIPanel;

        private void Start()
        {
            PoseManager.OnLeftHandStartGrab += HideUI;
            PoseManager.OnLeftHandStartPinch += HideUI;
            PoseManager.OnLeftHandStartPoint += HideUI;

            PoseManager.OnLeftHandStopGrab += ShowUI;
            PoseManager.OnLeftHandStopPinch += ShowUI;
            PoseManager.OnLeftHandStopPoint += ShowUI;
        }

        private void HideUI()
        {
            UIPanel.SetActive(false);
        }

        private void ShowUI()
        {
            UIPanel.SetActive(true);
        }
    }
}