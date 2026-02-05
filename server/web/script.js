/* ============================================
   LOVO - 모듈형 가구 주문 시스템
   ============================================ */

// ============================================
// 더미 데이터
// ============================================

const furnitureData = {
    bed: {
        name: '침대',
        description: '편안한 수면을 위한 모듈형 침대. 원하는 크기와 스타일로 커스터마이징하세요.',
        materials: [
            {
                id: 'bed-frame',
                name: '침대 프레임',
                options: [
                    { type: 'material', label: '재질', choices: ['원목', '합판'] },
                ]
            },
            {
                id: 'legs',
                name: '침대 다리',
                options: [
                    { type: 'color', label: '색상', choices: ['A', 'B'] },

                ]
            },
            {
                id: 'kitset',
                name: '작업키트',
                options: [
                    { type: 'material', label: '필수', choices: ['작업키트'] },

                ]
            }

        ]
    },
    chair: {
        name: '의자',
        description: '공간에 맞는 현대적인 모듈형 의자. 편안함과 디자인을 동시에 만족하세요.',
        materials: [
            {
                id: 'chair-frame',
                name: '의자 프레임',
                options: [
                    { type: 'material', label: '재질', choices: ['원목', '합판'] },
                ]
            },
            {
                id: 'legs',
                name: '의자 다리',
                options: [
                    { type: 'color', label: '색상', choices: ['A', 'B'] },

                ]
            },
            {
                id: 'wheels',
                name: '바퀴',
                options: [
                    { type: 'material', label: '재질', choices: ['고무', '없음'] },

                ]
            },
            {
                id: 'kitset',
                name: '작업키트',
                options: [
                    { type: 'material', label: '필수', choices: ['작업키트'] },

                ]
            }
        ]
    },
    desk: {
        name: '책상',
        description: '작업 공간을 위한 기능적 책상. 효율적인 업무 환경을 만드세요.',
        materials: [
            {
                id: 'desk-frame',
                name: '책상 프레임',
                options: [
                    { type: 'material', label: '재질', choices: ['원목', '합판'] },
                ]
            },
            {
                id: 'desk-legs',
                name: '책상 다리',
                options: [
                    { type: 'color', label: '색상', choices: ['A', 'B'] },

                ]
            },
            {
                id: 'kitset',
                name: '작업키트',
                options: [
                    { type: 'material', label: '필수', choices: ['작업키트'] },

                ]
            }

        ]
    }
};

// ============================================
// API Functions
// ============================================

const API_BASE_URL = 'http://192.168.0.7:5000/api';

async function fetchMaterials() {
    try {
        const response = await fetch(`${API_BASE_URL}/materials`);
        if (!response.ok) throw new Error('Network response was not ok');
        return await response.json();
    } catch (error) {
        console.error('Error fetching materials:', error);
        return [];
    }
}

async function fetchRobots() {
    try {
        const response = await fetch(`${API_BASE_URL}/robots`);
        if (!response.ok) throw new Error('Network response was not ok');
        return await response.json();
    } catch (error) {
        console.error('Error fetching robots:', error);
        return [];
    }
}

async function submitOrder(orderData) {
    try {
        const response = await fetch(`${API_BASE_URL}/orders`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(orderData),
        });
        if (!response.ok) throw new Error('Order submission failed');
        return await response.json();
    } catch (error) {
        console.error('Error submitting order:', error);
        throw error;
    }
}

// ============================================
// 장바구니 관리
// ============================================

function initCart() {
    if (!localStorage.getItem('lovoCart')) {
        localStorage.setItem('lovoCart', JSON.stringify([]));
    }
}

function getCart() {
    initCart();
    return JSON.parse(localStorage.getItem('lovoCart')) || [];
}

function saveCart(cart) {
    localStorage.setItem('lovoCart', JSON.stringify(cart));
    updateCartBadge();
}

function addToCart(item) {
    const cart = getCart();
    const existingItem = cart.find(
        cartItem =>
            cartItem.furniture === item.furniture &&
            JSON.stringify(cartItem.options) === JSON.stringify(item.options)
    );

    if (existingItem) {
        existingItem.quantity += item.quantity;
    } else {
        cart.push(item);
    }

    saveCart(cart);
    showModal('성공', `장바구니에 추가되었습니다!`);
}

function removeFromCart(index) {
    const cart = getCart();
    cart.splice(index, 1);
    saveCart(cart);
    displayCart();
}

function updateCartItemQuantity(index, quantity) {
    const cart = getCart();
    if (quantity > 0) {
        cart[index].quantity = quantity;
        saveCart(cart);
        displayCart();
    }
}

function updateCartBadge() {
    const cart = getCart();
    const totalQuantity = cart.reduce((sum, item) => sum + item.quantity, 0);
    const badges = document.querySelectorAll('#cart-count');
    badges.forEach(badge => {
        badge.textContent = totalQuantity;
        badge.style.display = totalQuantity > 0 ? 'flex' : 'none';
    });
}

// ============================================
// 페이지 기능
// ============================================

function goToDetail(type) {
    window.location.href = `furniture-detail.html?type=${type}`;
}

function loadFurnitureDetail(type) {
    const furniture = furnitureData[type];

    if (!furniture) {
        window.location.href = 'furniture.html';
        return;
    }

    // 제목과 설명 설정
    document.getElementById('furniture-name').textContent = furniture.name;
    document.getElementById('furniture-description').textContent = furniture.description;

    // SVG 그리기 (가구별)
    drawFurnitureSVG(type);

    // 자재 폼 생성
    const container = document.getElementById('materials-container');
    container.innerHTML = '';

    furniture.materials.forEach(material => {
        const materialDiv = document.createElement('div');
        materialDiv.className = 'material-group';
        materialDiv.innerHTML = `<h4>${material.name}</h4>`;

        material.options.forEach(option => {
            const formGroup = document.createElement('div');
            formGroup.className = 'form-group';
            formGroup.innerHTML = `
                <label for="${material.id}-${option.type}">${option.label}</label>
                <select id="${material.id}-${option.type}" name="${material.id}-${option.type}">
                    <option value="">선택하세요</option>
                    ${option.choices.map(choice => `<option value="${choice}">${choice}</option>`).join('')}
                </select>
            `;
            materialDiv.appendChild(formGroup);
        });

        container.appendChild(materialDiv);
    });

    // 폼 제출 이벤트
    document.getElementById('order-form').addEventListener('submit', function (e) {
        e.preventDefault();

        const formData = new FormData(this);
        const options = {};
        let allSelected = true;

        formData.forEach((value, key) => {
            if (!value) {
                allSelected = false;
            }
            options[key] = value;
        });

        if (!allSelected) {
            showModal('경고', '모든 옵션을 선택해주세요.');
            return;
        }

        const quantity = parseInt(document.getElementById('total-quantity').value) || 1;

        const cartItem = {
            furniture: furniture.name,
            furnitureType: type,
            materials: furniture.materials.map(m => m.name),
            options: options,
            quantity: quantity,
            timestamp: new Date().getTime()
        };

        addToCart(cartItem);
    });
}

function drawFurnitureSVG(type) {
    const svg = document.getElementById('furniture-svg');
    // SVG 요소 대신 이미지를 넣기 위해 부모 요소를 활용하거나 innerHTML을 교체
    // 여기서는 svg 태그 내에 foreignObject를 쓰거나, 아니면 아예 이미지를 감싸는 div로 처리하는게 낫지만,
    // 기존 구조 유지를 위해 이미지를 포함하는 HTML 문자열로 교체합니다.

    let imagePath = '';
    switch (type) {
        case 'bed':
            imagePath = 'images/bed.png';
            break;
        case 'chair':
            imagePath = 'images/chair.png';
            break;
        case 'desk':
            imagePath = 'images/desk.png';
            break;
    }

    if (imagePath) {
        // SVG 태그를 이미지 태그로 교체하기 위해 부모 요소를 찾음
        const parent = svg.parentElement;
        parent.innerHTML = `<img src="${imagePath}" alt="${type}" style="width: 100%; height: 100%; object-fit: contain;">`;
    }
}


// ============================================
// 자재 테이블 표시
// ============================================

async function displayMaterialsTable() {
    const tbody = document.getElementById('materials-tbody');
    // Only show loading on initial load if empty, to avoid flickering
    if (!tbody.hasChildNodes()) {
        tbody.innerHTML = '<tr><td colspan="4">로딩 중...</td></tr>';
    }

    const materials = await fetchMaterials();

    if (materials.length === 0) {
        tbody.innerHTML = '<tr><td colspan="4">데이터를 불러올 수 없습니다.</td></tr>';
        return;
    }

    // Clear current content before rebuilding
    // Note: For better UX, we should diff/update, but simple rebuild is okay for now.
    tbody.innerHTML = '';

    materials.forEach(material => {
        const row = document.createElement('tr');
        let statusClass = 'status-good';
        let statusText = '재고 충분';

        // Assuming API returns { name, quantity, unit, minStock }
        // If minStock is missing, default to 10
        const minStock = material.minStock || 10;

        if (material.quantity === 0) {
            statusClass = 'status-out';
            statusText = '품절';
        } else if (material.quantity <= minStock) {
            statusClass = 'status-low';
            statusText = '재고 부족';
        }

        row.innerHTML = `
            <td>${material.name}</td>
            <td>${material.quantity}</td>
            <td>${material.unit || '개'}</td>
            <td><span class="${statusClass}">${statusText}</span></td>
        `;
        tbody.appendChild(row);
    });
}

// ============================================
// 장바구니 표시
// ============================================

function displayCart() {
    const cart = getCart();
    const emptyMessage = document.getElementById('empty-cart-message');
    const cartContent = document.getElementById('cart-content');
    const tbody = document.getElementById('cart-tbody');

    if (cart.length === 0) {
        emptyMessage.style.display = 'block';
        cartContent.style.display = 'none';
        return;
    }

    emptyMessage.style.display = 'none';
    cartContent.style.display = 'grid';

    tbody.innerHTML = '';

    let totalItems = new Set();
    let totalQuantity = 0;

    cart.forEach((item, index) => {
        totalItems.add(item.furniture);
        totalQuantity += item.quantity;

        const optionsString = Object.entries(item.options)
            .map(([key, value]) => `${key}: ${value}`)
            .join(', ');

        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${item.furniture}</td>
            <td>${item.materials.join(', ')}</td>
            <td style="font-size: 12px; max-width: 200px; word-break: break-word;">
                ${optionsString}
            </td>
            <td>
                <input type="number" class="qty-input" value="${item.quantity}" min="1" 
                    onchange="updateCartItemQuantity(${index}, parseInt(this.value))">
            </td>
            <td>
                <button class="delete-button" onclick="removeFromCart(${index})">삭제</button>
            </td>
        `;
        tbody.appendChild(row);
    });

    document.getElementById('total-items-count').textContent = totalItems.size;
    document.getElementById('total-quantity-count').textContent = totalQuantity;

    updateCartBadge();
}

// ============================================
// 모달 표시
// ============================================

function showModal(title, message) {
    // 기존 모달이 있으면 제거
    const existingModal = document.getElementById('modal');
    if (existingModal) {
        existingModal.remove();
    }

    const modal = document.createElement('div');
    modal.id = 'modal';
    modal.className = 'modal';
    modal.innerHTML = `
        <div class="modal-content">
            <h2>${title}</h2>
            <p>${message}</p>
            <div class="modal-buttons">
                <button class="modal-ok" onclick="closeModal()">확인</button>
            </div>
        </div>
    `;

    document.body.appendChild(modal);
    modal.style.display = 'block';

    window.closeModal = function () {
        modal.style.display = 'none';
        modal.remove();
    };

    window.onclick = function (event) {
        if (event.target === modal) {
            modal.style.display = 'none';
            modal.remove();
        }
    };
}

// ============================================
// 주문 완료
// ============================================

async function confirmOrder() {
    const cart = getCart();
    if (cart.length === 0) {
        showModal('안내', '장바구니가 비어있습니다.');
        return;
    }

    const totalQuantity = cart.reduce((sum, item) => sum + item.quantity, 0);

    // Prepare data for API
    const orderData = {
        items: cart,
        timestamp: new Date().toISOString()
    };

    try {
        showModal('처리 중', '주문을 전송하고 있습니다...');
        const result = await submitOrder(orderData);

        showModal('주문 완료', `주문번호: ${result.orderId}\n총 ${cart.length}개의 가구, ${totalQuantity}개의 자재로 주문이 완료되었습니다!\n\n감사합니다!`);

        setTimeout(() => {
            localStorage.setItem('lovoCart', JSON.stringify([]));
            updateCartBadge();
            window.location.href = 'furniture.html';
        }, 2000);

    } catch (error) {
        showModal('오류', '주문 전송 중 오류가 발생했습니다.\n서버 상태를 확인해주세요.');
    }
}

// ============================================
// 초기 로드
// ============================================

document.addEventListener('DOMContentLoaded', function () {
    initCart();
    updateCartBadge();

    // Initial load
    if (document.getElementById('materials-tbody')) {
        displayMaterialsTable();
        // Poll every 3 seconds for real-time updates
        setInterval(displayMaterialsTable, 3000);
    }

    // Monitor load
    if (document.getElementById('map-area')) {
        initMonitor();
    }
});

// ============================================
// 로봇 관제 (Monitor)
// ============================================

async function initMonitor() {
    const mapArea = document.getElementById('map-area');
    const robotList = document.getElementById('robot-list');

    if (!mapArea || !robotList) return;

    // 초기 로드
    updateMonitor();

    // 1초마다 갱신
    setInterval(updateMonitor, 1000);
}

async function updateMonitor() {
    const robots = await fetchRobots();
    const mapArea = document.getElementById('map-area');
    const robotList = document.getElementById('robot-list');

    // 1. Robot Markers
    const existingMarkers = document.querySelectorAll('.robot-marker');
    existingMarkers.forEach(el => el.remove());

    robots.forEach(robot => {
        const marker = document.createElement('div');
        marker.className = `robot-marker ${robot.robot_kind === 'ARM' ? 'arm' : 'pinky'}`;

        // Scale and Offset
        const scale = 12;
        const x = (robot.pose_x || 0) * scale + 50;
        const y = (robot.pose_y || 0) * scale + 50;

        marker.style.left = `${x}px`;
        marker.style.top = `${y}px`;
        marker.innerHTML = robot.robot_kind === 'ARM' ? '🦾' : '🤖';

        const label = document.createElement('div');
        label.className = 'robot-label';
        label.innerText = robot.robot_role;
        marker.appendChild(label);

        mapArea.appendChild(marker);
    });

    // 2. Status Cards
    robotList.innerHTML = '';
    robots.forEach(robot => {
        const card = document.createElement('div');
        card.className = 'robot-card';
        const statusClass = `status-${robot.action_state.toLowerCase()}`;

        card.innerHTML = `
            <h4>${robot.robot_role} <span class="status-badge ${statusClass}">${robot.action_state}</span></h4>
            <p><strong>배터리:</strong> ${robot.battery_percent || 100}%</p>
            <p><strong>위치:</strong> (${robot.pose_x?.toFixed(1)}, ${robot.pose_y?.toFixed(1)})</p>
        `;
        robotList.appendChild(card);
    });
}
